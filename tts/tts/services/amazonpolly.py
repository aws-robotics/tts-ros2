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

import json
import os
import sys
import wave
import traceback
from boto3 import Session
from botocore.session import get_session
from botocore.exceptions import UnknownServiceError
from contextlib import closing
from optparse import OptionParser

from tts_interfaces.srv import Polly

import rclpy
from rclpy.node import Node


class AmazonPolly:
    """
    A TTS engine that can be used in two different ways.

    Usage
    -----

    1. It can run as a ROS service node.

    Start a polly node::

        $ ros2 run tts polly_server

    Call the service from command line::

        $ ros2 service call /polly tts_interfaces/Polly "{polly_action: 'SynthesizeSpeech', text: 'hello'}"

    Call the service programmatically::

        from tts_interfaces.srv import Polly

        rclpy.init()
        node = rclpy.create_node('ClientNode')
        client = node.create_client(Polly, 'polly')
        client.wait_for_service()
        text = 'Mary has a little lamb, little lamb, little lamb.'
        request = Polly.Request(polly_action='SynthesizeSpeech', text=text)
        future = client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                res = future.result()
                ...
        ...
        node.destroy_node()
        rclpy.shutdown()

    2. It can also be used as a normal python class::

        AmazonPolly().synthesize(text='hi polly')

    Polly.Request supports many parameters, but the majority of the users can safely ignore most of them and just
    use the vanilla version which involves only one argument, ``text``.

    If in some use cases more control is needed, SSML will come handy. Example::

        AmazonPolly().synthesize(
            text='<speak>Mary has a <amazon:effect name="whispered">little lamb.</amazon:effect></speak>',
            text_type='ssml'
        )

    A user can also control the voice, output format and so on. Example::

        AmazonPolly().synthesize(
            text='<speak>Mary has a <amazon:effect name="whispered">little lamb.</amazon:effect></speak>',
            text_type='ssml',
            voice_id='Joey',
            output_format='mp3',
            output_path='/tmp/blah'
        )


    Service Parameters
    ------------------

    Among the parameters defined in Polly.srv, the following are supported while others are reserved for future.

    * polly_action : currently only ``SynthesizeSpeech`` is supported
    * text : the text to speak
    * text_type : can be either ``text`` (default) or ``ssml``
    * voice_id : any voice id supported by Amazon Polly, default is Joanna
    * output_format : pcm (default), mp3 or ogg
    * output_path : where the audio file is saved
    * sample_rate : default is 16000 for pcm or 22050 for mp3 and ogg

    The following are the reserved ones. Note that ``language_code`` is rarely needed (this may seem counter-intuitive).
    See official Amazon Polly documentation for details (link can be found below).

    * language_code
    * lexicon_content
    * lexicon_name
    * lexicon_names
    * speech_mark_types
    * max_results
    * next_token
    * sns_topic_arn
    * task_id
    * task_status
    * output_s3_bucket_name
    * output_s3_key_prefix
    * include_additional_language_codes


    Links
    -----

    Amazon Polly documentation: https://docs.aws.amazon.com/polly/latest/dg/API_SynthesizeSpeech.html

    """

    def __init__(self, aws_access_key_id=None, aws_secret_access_key=None, aws_session_token=None, region_name=None,
                 logger=None):
        if region_name is None:
            region_name = 'us-west-2'

        # for maximum flexibility, logger can be None, in which case nothing will be logged
        self.logger = logger

        self.polly = self._get_polly_client(aws_access_key_id, aws_secret_access_key, aws_session_token, region_name)
        self.default_text_type = 'text'
        self.default_voice_id = 'Joanna'
        self.default_output_format = 'pcm'
        self.default_output_folder = '.'
        self.default_output_file_basename = 'output'

    def loginfo(self, s):
        if self.logger:
            self.logger.info(s)

    def logerr(self, s):
        if self.logger:
            self.logger.error(s)

    def _get_polly_client(self, aws_access_key_id=None, aws_secret_access_key=None, aws_session_token=None,
                          region_name=None, with_service_model_patch=False):
        # Note we get a new botocore session each time this function is called
        # to avoid potential problems caused by inner state of the session.
        botocore_session = get_session()

        if with_service_model_patch:
            # Older versions of botocore don't have polly. We can possibly fix it by appending
            # extra path with polly service model files to the search path.
            current_dir = os.path.dirname(os.path.abspath(__file__))
            service_model_path = os.path.join(current_dir, 'data', 'models')
            botocore_session.set_config_variable('data_path', service_model_path)
            self.loginfo('patching service model data path: {}'.format(service_model_path))

        botocore_session.user_agent_extra = self._generate_user_agent_suffix()

        session = Session(aws_access_key_id=aws_access_key_id, aws_secret_access_key=aws_secret_access_key,
                          aws_session_token=aws_session_token, region_name=region_name,
                          botocore_session=botocore_session)

        try:
            return session.client("polly")
        except UnknownServiceError:
            # the first time we reach here, we try to fix the problem
            if not with_service_model_patch:
                return self._get_polly_client(aws_access_key_id, aws_secret_access_key, aws_session_token, region_name,
                                              with_service_model_patch=True)
            else:
                # we have tried our best, time to panic
                self.logerr('Amazon Polly is not available. Please install the latest boto3.')
                raise

    def _generate_user_agent_suffix(self):
        exec_env = 'AWS_RoboMaker'
        ros_distro = 'ROS2'
        ros_version = 'Unknown_ROS_VERSION'
        return 'exec-env/{} ros-{}/{}'.format(exec_env, ros_distro, ros_version)

    def _pcm2wav(self, audio_data, wav_filename, sample_rate):
        # per Amazon Polly official doc, the pcm in a signed 16-bit, 1 channel (mono), little-endian format.
        wavf = wave.open(wav_filename, 'w')
        wavf.setframerate(int(sample_rate))
        wavf.setnchannels(1)  # 1 channel
        wavf.setsampwidth(2)  # 2 bytes == 16 bits
        wavf.writeframes(audio_data)
        wavf.close()

    def _make_audio_file_fullpath(self, output_path, output_format):
        """
        Make a full path for audio file based on given output path and format.

        If ``output_path`` doesn't have a path, current path is used.

        :param output_path: the output path received
        :param output_format: the audio format, e.g., mp3, ogg_vorbis, pcm
        :return: a full path for the output audio file. File ext will be constructed from audio format.
        """
        head, tail = os.path.split(output_path)
        if not head:
            head = self.default_output_folder
        if not tail:
            tail = self.default_output_file_basename

        file_ext = {'pcm': '.wav', 'mp3': '.mp3', 'ogg_vorbis': '.ogg'}[output_format.lower()]
        if not tail.endswith(file_ext):
            tail += file_ext

        return os.path.realpath(os.path.join(head, tail))

    def _synthesize_speech_and_save(self, request):
        """
        Call Amazon Polly and writes the returned audio data to a local file.

        To make it practical, three things will be returned in a JSON form string, which are audio file path,
        audio type and Amazon Polly response metadata.

        If the Amazon Polly call fails, audio file name will be an empty string and audio type will be "N/A".

        Please see https://boto3.readthedocs.io/reference/services/polly.html#Polly.Client.synthesize_speech
        for more details on Amazon Polly API.

        :param request: an instance of Polly.Request
        :return: a string in JSON form with two attributes, "Audio File" and "Amazon Polly Response".
        """
        kws = {
            'LexiconNames': request.lexicon_names if request.lexicon_names else [],
            'OutputFormat': request.output_format if request.output_format else self.default_output_format,
            'SampleRate': request.sample_rate,
            'SpeechMarkTypes': request.speech_mark_types if request.speech_mark_types else [],
            'Text': request.text,
            'TextType': request.text_type if request.text_type else self.default_text_type,
            'VoiceId': request.voice_id if request.voice_id else self.default_voice_id
        }

        if not kws['SampleRate']:
            kws['SampleRate'] = '16000' if kws['OutputFormat'].lower() == 'pcm' else '22050'

        self.loginfo('Amazon Polly Request: {}'.format(kws))
        response = self.polly.synthesize_speech(**kws)
        self.loginfo('Amazon Polly Response: {}'.format(response))

        if "AudioStream" in response:
            audiofile = self._make_audio_file_fullpath(request.output_path, kws['OutputFormat'])
            self.loginfo('will save audio as {}'.format(audiofile))

            with closing(response["AudioStream"]) as stream:
                if kws['OutputFormat'].lower() == 'pcm':
                    self._pcm2wav(stream.read(), audiofile, kws['SampleRate'])
                else:
                    with open(audiofile, "wb") as f:
                        f.write(stream.read())

            audiotype = response['ContentType']
        else:
            audiofile = ''
            audiotype = 'N/A'

        return json.dumps({
            'Audio File': audiofile,
            'Audio Type': audiotype,
            'Amazon Polly Response Metadata': str(response['ResponseMetadata'])
        })

    def _dispatch(self, request):
        """
        Amazon Polly supports a number of APIs. This will call the right one based on the content of request.

        Currently "SynthesizeSpeech" is the only recognized action. Basically this method just delegates the work
        to ``self._synthesize_speech_and_save`` and returns the result as is. It will simply raise if a different
        action is passed in.

        :param request: an instance of Polly.Request
        :return: whatever returned by the delegate
        """
        actions = {
            'SynthesizeSpeech': self._synthesize_speech_and_save
            # ... more actions could go in here ...
        }

        if request.polly_action not in actions:
            raise RuntimeError('bad or unsupported Amazon Polly action: "' + request.polly_action + '".')

        return actions[request.polly_action](request)

    def node_request_handler(self, request, response):
        """
        Process service request. The callback function.

        It never raises. If anything unexpected happens, it will return a Polly.Response with details of the exception.

        :param request: an instance of Polly.Request
        :param response: an instance of Polly.Response
        :return: a Polly.Response
        """
        self.loginfo('Amazon Polly Request: {}'.format(request))

        try:
            result = self._dispatch(request)
            self.loginfo('will return {}'.format(result))
            response.result = result
            return response
        except Exception as e:
            current_dir = os.path.dirname(os.path.abspath(__file__))  # todo: make use of pkg_resources
            exc_type = sys.exc_info()[0]

            # not using `issubclass(exc_type, ConnectionError)` for the condition below because some versions
            # of urllib3 raises exception when doing `from requests.exceptions import ConnectionError`
            error_ogg_filename = 'connerror.ogg' if 'ConnectionError' in exc_type.__name__ else 'error.ogg'

            error_details = {
                'Audio File': os.path.join(current_dir, 'data', error_ogg_filename),
                'Audio Type': 'ogg',
                'Exception': {
                    'Type': str(exc_type),
                    'Module': exc_type.__module__,
                    'Name': exc_type.__name__,
                    'Value': str(e),
                },
                'Traceback': traceback.format_exc()
            }

            error_str = json.dumps(error_details)
            self.logerr(error_str)
            response.result = error_str
            return response

    def synthesize(self, **kws):
        """
        Call this method if you want to use polly but don't want to start a node.

        :param kws: input as defined in Polly.srv
        :return: a string in JSON form with detailed information, success or failure
        """
        request = Polly.Request(polly_action='SynthesizeSpeech', **kws)
        response = Polly.Response()
        return self.node_request_handler(request, response)


class AmazonPollyNode(Node):

    def __init__(self, node_name, service_name):
        super().__init__(node_name)
        self.polly = AmazonPolly(logger=self.get_logger())
        self.srv = self.create_service(Polly, service_name, self.polly_callback)

    def polly_callback(self, request, response):
        return self.polly.node_request_handler(request, response)


def main():
    usage = '''usage: %prog [options]
    '''

    parser = OptionParser(usage)

    parser.add_option("-n", "--node-name", dest="node_name", default='polly_node',
                      help="name of the ROS node",
                      metavar="NODE_NAME")
    parser.add_option("-s", "--service-name", dest="service_name", default='polly',
                      help="name of the ROS service",
                      metavar="SERVICE_NAME")

    (options, args) = parser.parse_args()

    node_name = options.node_name
    service_name = options.service_name

    rclpy.init()

    node = AmazonPollyNode(node_name, service_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down polly server')
    except BaseException:
        print('Exception in polly server:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
