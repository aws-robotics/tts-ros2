# Copyright (c) 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

import hashlib
import json
import os
import time
from optparse import OptionParser

import rclpy
from rclpy.node import Node
from tts_interfaces.srv import Polly
from tts_interfaces.srv import Synthesizer


class SpeechSynthesizer:
    """
    This class serves as a ROS service node that should be an entry point of a TTS task.

    Although the current implementation uses Amazon Polly as the synthesis engine, it is not hard to let it support
    more heterogeneous engines while keeping the API the same.

    In order to support a variety of engines, the Synthesizer.Request was designed with flexibility in mind. It
    has two fields: text and metadata. Both are strings. In most cases, a user can ignore the metadata and call
    the service with some plain text. If the use case needs any control or engine-specific feature, the extra
    information can be put into the JSON-form metadata. This class will use the information when calling the engine.

    The decoupling of the synthesizer and the actual synthesis engine will benefit the users in many ways.

    First, a user will be able to use a unified interface to do the TTS job and have the freedom to use different
    engines available with no or very little change from the client side.

    Second, by applying some design patterns, the synthesizer can choose an engine dynamically. For example, a user
    may prefer to use Amazon Polly but is also OK with an offline solution when network is not reliable.

    Third, engines can be complicated, thus difficult to use. As an example, Amazon Polly supports dozens of parameters
    and is able to accomplish nontrivial synthesis jobs, but majority of the users never need those features. This
    class provides a clean interface with two parameters only, so that it is much easier and pleasant to use. If by
    any chance the advanced features are required, the user can always leverage the metadata field or even go to the
    backend engine directly.

    Also, from an engineering perspective, simple and decoupled modules are easier to maintain.

    This class supports two modes of using polly. It can either call a service node or use AmazonPolly as a library.

    Start the service node::

        $ ros2 run tts synthesizer_server  # use default configuration
        $ ros2 run tts synthesizer_server -e POLLY_LIBRARY  # will not call polly service node

    Call the service::

        $ ros2 service call /synthesizer tts_interfaces/Synthesizer "{text: 'hello'}"

        $ md='{"text_type": "ssml"}'
        $ ros2 service call /synthesizer tts_interfaces/Synthesizer "{text: '<speak>hello</speak>', metadata: '$md'}"
    """

    class PollyViaNode:
        """Using a dedicated node to communicate with polly node."""

        def __init__(self, **kwargs):
            self.service_name = kwargs['polly_service_name']
            self.node = rclpy.create_node('synthesizer_node_polly_client')
            self.polly_client = self.node.create_client(Polly, self.service_name)
            self.logger = self.node.get_logger()

        def __call__(self, **kwargs):
            self.logger.info('Will call service {}'.format(self.service_name))

            request = Polly.Request(polly_action='SynthesizeSpeech', **kwargs)

            self.polly_client.wait_for_service()
            self.logger.info('Service is available')

            future = self.polly_client.call_async(request)

            while rclpy.ok():
                rclpy.spin_once(self.node)
                if future.done():
                    if future.result() is not None:
                        self.logger.info('Done calling {}. Result {}'.format(self.service_name, future.result()))
                        return future.result()
                    else:
                        self.logger.error('Failed calling %s: %r' % (self.service_name, future.exception()))
                        return future.exception()
                self.logger.info('Waiting for service to be done.')

    class PollyDirect:
        """Using library to communicate with polly node directly."""

        def __init__(self, **kwargs):
            self.logger = kwargs['logger']

        def __call__(self, **kwargs):
            self.logger.info('will import amazonpolly.AmazonPolly')
            from .amazonpolly import AmazonPolly
            amazon_polly = AmazonPolly(logger=self.logger)
            return amazon_polly.synthesize(**kwargs)

    ENGINES = {
        'POLLY_SERVICE': PollyViaNode,
        'POLLY_LIBRARY': PollyDirect,
    }

    class BadEngineError(NameError):
        pass

    def __init__(self, logger, engine='POLLY_SERVICE', engine_service_name='polly',
                 default_text_type='text', default_voice_id='Joanna', default_output_format='pcm'):
        self.logger = logger

        if engine not in self.ENGINES:
            msg = 'bad engine {} which is not one of {}'.format(engine, ', '.join(SpeechSynthesizer.ENGINES.keys()))
            raise SpeechSynthesizer.BadEngineError(msg)

        engine_kwargs = {}
        # more advanced plugin techniques would be overkilling so plain old dict is used here
        if engine == 'POLLY_SERVICE':
            engine_kwargs.update({
                'polly_service_name': engine_service_name
            })
        else:
            engine_kwargs.update({
                'logger': self.logger
            })
        self.engine = self.ENGINES[engine](**engine_kwargs)

        self.default_text_type = default_text_type
        self.default_voice_id = default_voice_id
        self.default_output_format = default_output_format

    def _call_engine(self, **kw):
        """
        Call engine to do the job.

        If no output path is found from input, the audio file will be put into /tmp and the file name will have
        a prefix of the md5 hash of the text.

        :param kw: what AmazonPolly needs to synthesize
        :return: response from AmazonPolly
        """
        if 'output_path' not in kw:
            tmp_filename = hashlib.md5(kw['text'].encode('utf-8')).hexdigest()
            tmp_filepath = os.path.join(os.sep, 'tmp', 'voice_{}_{}'.format(tmp_filename, str(time.time())))
            kw['output_path'] = os.path.abspath(tmp_filepath)
        self.logger.info('audio will be saved as {}'.format(kw['output_path']))

        return self.engine(**kw)

    def _parse_request_or_raise(self, request):
        """
        Transform a request to a dict, or raise if request is malformed.

        :param request: an instance of Synthesizer.Request
        :return: a dict
        """
        md = json.loads(request.metadata) if request.metadata else {}

        md['output_format'] = md.get('output_format', self.default_output_format)
        md['voice_id'] = md.get('voice_id', self.default_voice_id)
        md['sample_rate'] = md.get('sample_rate', '16000' if md['output_format'].lower() == 'pcm' else '22050')
        md['text_type'] = md.get('text_type', self.default_text_type)
        md['text'] = request.text

        return md

    def node_request_handler(self, request, response):
        """
         Process service request. The callback function.

        It never raises. If anything unexpected happens, it will return a Synthesizer.Response with the exception.

        :param request: an instance of Synthesizer.Request
        :param response: an instance of Synthesizer.Response
        :return: the Synthesizer.Response
        """
        self.logger.info(str(request))
        try:
            kws = self._parse_request_or_raise(request)
            response.result = self._call_engine(**kws).result
        except Exception as e:
            import traceback
            response.result = 'Exception: {}.\nTraceback:\n{}'.format(e, traceback.format_exc())

        self.logger.info('Will send response: {}'.format(response.result))
        return response


class SynthesizerNode(Node):

    def __init__(self, node_name='synthesizer_node', service_name='synthesizer_service',
                 engine='POLLY_SERVICE', engine_service_name='polly'):
        super().__init__(node_name)
        self.synthesizer = SpeechSynthesizer(engine=engine,
                                             engine_service_name=engine_service_name,
                                             logger=self.get_logger())
        self.srv = self.create_service(Synthesizer, service_name, self._callback)

    def _callback(self, request, response):
        return self.synthesizer.node_request_handler(request, response)


def main():
    usage = '''usage: %prog [options]
    '''

    parser = OptionParser(usage)

    parser.add_option("-n", "--node-name", dest="node_name", default='synthesizer_node',
                      help="name of the ROS node",
                      metavar="NODE_NAME")
    parser.add_option("-s", "--service-name", dest="service_name", default='synthesizer',
                      help="name of the ROS service",
                      metavar="SERVICE_NAME")
    parser.add_option("-e", "--engine", dest="engine", default='POLLY_SERVICE',
                      help="name of the synthesis engine",
                      metavar="ENGINE")
    parser.add_option("-p", "--engine-service-name", dest="engine_service_name", default='polly',
                      help="name of the synthesis engine service",
                      metavar="ENGINE_SERVICE_NAME")

    (options, args) = parser.parse_args()

    node_name = options.node_name
    service_name = options.service_name
    engine = options.engine
    engine_service_name = options.engine_service_name

    rclpy.init()

    if engine.endswith('_SERVICE'):
        node = SynthesizerNode(node_name=node_name, service_name=service_name, engine=engine,
                               engine_service_name=engine_service_name)
    else:
        node = SynthesizerNode(node_name=node_name, service_name=service_name, engine=engine)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down synthesizer server')
    except BaseException:
        print('Exception in synthesizer server:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
