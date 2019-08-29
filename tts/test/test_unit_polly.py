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


class TestPolly(unittest.TestCase):

    def setUp(self):
        self.patchers = {
            s: patch(s) for s in
            (
                'rclpy.init',
                'rclpy.spin',
                'rclpy.shutdown',
            )
        }
        self.mocks = {s: p.start() for s, p in self.patchers.items()}

    def tearDown(self):
        for p in self.patchers.values():
            p.stop()

    @patch('tts.services.amazonpolly.Session')
    def test_init(self, boto3_session_class_mock):
        from tts.services.amazonpolly import AmazonPolly
        AmazonPolly()

        self.assertGreater(boto3_session_class_mock.call_count, 0)
        boto3_session_class_mock.return_value.client.assert_called_with('polly')

    @patch('tts.services.amazonpolly.Session')
    def test_defaults(self, boto3_session_class_mock):
        from tts.services.amazonpolly import AmazonPolly
        polly = AmazonPolly()

        self.assertGreater(boto3_session_class_mock.call_count, 0)
        boto3_session_class_mock.return_value.client.assert_called_with('polly')

        self.assertEqual('text', polly.default_text_type)
        self.assertEqual('pcm', polly.default_output_format)
        self.assertEqual('Joanna', polly.default_voice_id)
        self.assertEqual('.', polly.default_output_folder)
        self.assertEqual('output', polly.default_output_file_basename)

    @patch('tts.services.amazonpolly.Session')
    def test_good_synthesis_with_default_args(self, boto3_session_class_mock):
        pass
        boto3_session_obj_mock = MagicMock()
        boto3_polly_obj_mock = MagicMock()
        boto3_polly_response_mock = MagicMock()
        audio_stream_mock = MagicMock()
        fake_audio_stream_data = 'I am audio.'
        fake_audio_content_type = 'super tts'
        fake_boto3_polly_response_metadata = {'foo': 'bar'}

        boto3_session_class_mock.return_value = boto3_session_obj_mock
        boto3_session_obj_mock.client.return_value = boto3_polly_obj_mock
        boto3_polly_obj_mock.synthesize_speech.return_value = boto3_polly_response_mock
        audio_stream_mock.read.return_value = fake_audio_stream_data.encode('utf-8')
        d = {
            'AudioStream': audio_stream_mock,
            'ContentType': fake_audio_content_type,
            'ResponseMetadata': fake_boto3_polly_response_metadata
        }
        boto3_polly_response_mock.__contains__.side_effect = d.__contains__
        boto3_polly_response_mock.__getitem__.side_effect = d.__getitem__

        from tts.services.amazonpolly import AmazonPolly
        polly_under_test = AmazonPolly()

        self.assertGreater(boto3_session_class_mock.call_count, 0)
        boto3_session_obj_mock.client.assert_called_with('polly')

        res = polly_under_test.synthesize(output_format='ogg_vorbis', text='hello')

        expected_synthesize_speech_kwargs = {
            'LexiconNames': [],
            'OutputFormat': 'ogg_vorbis',
            'SampleRate': '22050',
            'SpeechMarkTypes': [],
            'Text': 'hello',
            'TextType': 'text',
            'VoiceId': 'Joanna',
        }
        boto3_polly_obj_mock.synthesize_speech.assert_called_with(**expected_synthesize_speech_kwargs)

        from tts_interfaces.srv import Polly
        self.assertTrue(isinstance(res, Polly.Response))

        import json
        print(res.result)
        j = json.loads(res.result)
        observed_audio_file_content = open(j['Audio File']).read()
        self.assertEqual(fake_audio_stream_data, observed_audio_file_content)

        self.assertEqual(fake_audio_content_type, j['Audio Type'])
        self.assertEqual(str(fake_boto3_polly_response_metadata), j['Amazon Polly Response Metadata'])

    @patch('tts.services.amazonpolly.Session')
    def test_polly_raises(self, boto3_session_class_mock):
        boto3_session_obj_mock = MagicMock()
        boto3_polly_obj_mock = MagicMock()
        boto3_polly_response_mock = MagicMock()
        audio_stream_mock = MagicMock()
        fake_audio_stream_data = 'I am audio.'
        fake_audio_content_type = 'super voice'
        fake_boto3_polly_response_metadata = {'foo': 'bar'}

        boto3_session_class_mock.return_value = boto3_session_obj_mock
        boto3_session_obj_mock.client.return_value = boto3_polly_obj_mock
        boto3_polly_obj_mock.synthesize_speech.side_effect = RuntimeError('Amazon Polly Exception')
        audio_stream_mock.read.return_value = fake_audio_stream_data
        d = {
            'AudioStream': audio_stream_mock,
            'ContentType': fake_audio_content_type,
            'ResponseMetadata': fake_boto3_polly_response_metadata
        }
        boto3_polly_response_mock.__contains__.side_effect = d.__contains__
        boto3_polly_response_mock.__getitem__.side_effect = d.__getitem__

        from tts.services.amazonpolly import AmazonPolly
        polly_under_test = AmazonPolly()

        self.assertGreater(boto3_session_class_mock.call_count, 0)
        boto3_session_obj_mock.client.assert_called_with('polly')

        res = polly_under_test.synthesize(text='hello')

        expected_synthesize_speech_kwargs = {
            'LexiconNames': [],
            'OutputFormat': 'pcm',
            'SampleRate': '16000',
            'SpeechMarkTypes': [],
            'Text': 'hello',
            'TextType': 'text',
            'VoiceId': 'Joanna',
        }
        boto3_polly_obj_mock.synthesize_speech.assert_called_with(**expected_synthesize_speech_kwargs)

        from tts_interfaces.srv import Polly
        self.assertTrue(isinstance(res, Polly.Response))

        import json
        j = json.loads(res.result)
        self.assertTrue('Exception' in j)
        self.assertTrue('Traceback' in j)

    @patch('tts.services.amazonpolly.AmazonPollyNode')
    def test_cli(self, amazon_polly_class_mock):
        import sys
        with patch.object(sys, 'argv', ['amazonpolly.py', '-n', 'pollynode']):
            from tts.services import amazonpolly
            amazonpolly.main()
            self.assertGreater(amazon_polly_class_mock.call_count, 0)


if __name__ == '__main__':
    unittest.main()
