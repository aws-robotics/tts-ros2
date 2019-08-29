#!/usr/bin/env python3

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

from unittest.mock import patch, MagicMock
import unittest
from pprint import pprint


class TestSynthesizer(unittest.TestCase):

    def setUp(self):
        pass
        import tts
        pprint(dir(tts.services))
        self.patchers = {
            s: patch(s) for s in
            (
                'rclpy.init',
                'rclpy.spin',
                'rclpy.shutdown',
                'tts.services.amazonpolly.AmazonPolly',
            )
        }
        self.mocks = {s: p.start() for s, p in self.patchers.items()}

    def tearDown(self):
        pass
        for p in self.patchers.values():
            p.stop()

    def test_init(self):
        from tts.services.synthesizer import SpeechSynthesizer
        speech_synthesizer = SpeechSynthesizer(logger=MagicMock(), engine='POLLY_LIBRARY')
        self.assertEqual('text', speech_synthesizer.default_text_type)

    def test_good_synthesis_with_mostly_default_args_using_polly_lib(self):
        polly_obj_mock = MagicMock()
        polly_class_mock = self.mocks['tts.services.amazonpolly.AmazonPolly']
        polly_class_mock.return_value = polly_obj_mock

        test_text = 'hello'
        test_metadata = '''
            {
                "output_path": "/tmp/test"
            }
        '''
        expected_polly_synthesize_args = {
            'output_format': 'pcm',
            'voice_id': 'Joanna',
            'sample_rate': '16000',
            'text_type': 'text',
            'text': test_text,
            'output_path': "/tmp/test"
        }

        from tts.services.synthesizer import SpeechSynthesizer
        from tts_interfaces.srv import Synthesizer
        speech_synthesizer = SpeechSynthesizer(logger=MagicMock(), engine='POLLY_LIBRARY')
        request = Synthesizer.Request(text=test_text, metadata=test_metadata)
        response = Synthesizer.Response(result='')
        polly_obj_mock.synthesize.return_value = response

        response = speech_synthesizer.node_request_handler(request, response)

        self.assertGreater(polly_class_mock.call_count, 0)
        polly_obj_mock.synthesize.assert_called_with(**expected_polly_synthesize_args)

        self.assertEqual(response.result, polly_obj_mock.synthesize.return_value.result)

    def test_synthesis_with_bad_metadata_using_polly_lib(self):
        polly_class_mock = self.mocks['tts.services.amazonpolly.AmazonPolly']

        polly_obj_mock = MagicMock()
        polly_class_mock.return_value = polly_obj_mock

        test_text = 'hello'
        test_metadata = '''I am no JSON'''

        from tts.services.synthesizer import SpeechSynthesizer
        from tts_interfaces.srv import Synthesizer
        speech_synthesizer = SpeechSynthesizer(logger=MagicMock(), engine='POLLY_LIBRARY')
        request = Synthesizer.Request(text=test_text, metadata=test_metadata)
        response = Synthesizer.Response()
        response = speech_synthesizer.node_request_handler(request, response)

        self.assertTrue(response.result.startswith('Exception: '))

    def test_bad_engine(self):
        polly_class_mock = self.mocks['tts.services.amazonpolly.AmazonPolly']
        polly_obj_mock = MagicMock()
        polly_class_mock.return_value = polly_obj_mock

        ex = None

        from tts.services.synthesizer import SpeechSynthesizer

        try:
            SpeechSynthesizer(logger=MagicMock(), engine='NON-EXIST ENGINE')
        except Exception as e:
            ex = e

        print(ex)

        self.assertTrue(isinstance(ex, SpeechSynthesizer.BadEngineError))

    def test_cli_help_message(self):
        import os
        source_file_dir = os.path.dirname(os.path.abspath(__file__))
        synthersizer_path = os.path.join(source_file_dir, '..', 'tts', 'services', 'synthesizer.py')
        import subprocess
        o = subprocess.check_output(['python3', synthersizer_path, '-h']).decode('utf-8')
        self.assertTrue(str(o).startswith('Usage: '))

    @patch('tts.services.synthesizer.SynthesizerNode')
    def test_cli_engine_dispatching_1(self, speech_synthesizer_class_mock):
        rclpy_init = self.mocks['rclpy.init']
        rclpy_spin = self.mocks['rclpy.spin']
        rclpy_shutdown = self.mocks['rclpy.shutdown']
        import sys
        with patch.object(sys, 'argv', ['synthesizer.py']):
            from tts.services.synthesizer import main as synthesizer_main
            synthesizer_main()
            speech_synthesizer_class_mock.assert_called_with(engine='POLLY_SERVICE',
                                                             engine_service_name='polly',
                                                             node_name='synthesizer_node',
                                                             service_name='synthesizer')
            rclpy_init.assert_called()
            rclpy_spin.assert_called()
            rclpy_shutdown.assert_called()

    @patch('tts.services.synthesizer.SynthesizerNode')
    def test_cli_engine_dispatching_2(self, speech_synthesizer_class_mock):
        rclpy_init = self.mocks['rclpy.init']
        rclpy_spin = self.mocks['rclpy.spin']
        rclpy_shutdown = self.mocks['rclpy.shutdown']
        import sys
        with patch.object(sys, 'argv', ['synthesizer.py', '-e', 'POLLY_LIBRARY']):
            from tts.services.synthesizer import main as synthesizer_main
            synthesizer_main()
            speech_synthesizer_class_mock.assert_called_with(engine='POLLY_LIBRARY',
                                                             node_name='synthesizer_node',
                                                             service_name='synthesizer')
            rclpy_init.assert_called()
            rclpy_spin.assert_called()
            rclpy_shutdown.assert_called()

    @patch('tts.services.synthesizer.SynthesizerNode')
    def test_cli_engine_dispatching_3(self, speech_synthesizer_class_mock):
        rclpy_init = self.mocks['rclpy.init']
        rclpy_spin = self.mocks['rclpy.spin']
        rclpy_shutdown = self.mocks['rclpy.shutdown']
        import sys
        with patch.object(sys, 'argv', ['synthesizer_node.py', '-p', 'apolly']):
            from tts.services.synthesizer import main as synthesizer_main
            synthesizer_main()
            speech_synthesizer_class_mock.assert_called_with(engine='POLLY_SERVICE',
                                                             engine_service_name='apolly',
                                                             node_name='synthesizer_node',
                                                             service_name='synthesizer')
            rclpy_init.assert_called()
            rclpy_spin.assert_called()
            rclpy_shutdown.assert_called()


if __name__ == '__main__':
    unittest.main()
