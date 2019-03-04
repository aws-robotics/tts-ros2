#!/usr/bin/env python

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

from __future__ import print_function

import json
import unittest

import rclpy

from tts_interfaces.srv import Polly
from tts_interfaces.srv import Synthesizer


PKG = 'tts'
NAME = 'amazonpolly'


class TestPlainText(unittest.TestCase):
    pass

    def test_plain_text_to_wav_via_polly_node(self):
        rclpy.init()
        node = rclpy.create_node('integtest')
        client = node.create_client(Polly, 'polly')

        retries = 0
        while not client.wait_for_service(timeout_sec=1.0):
            retries += 1
            self.failIf(retries > 3, 'service is not available')

        test_text = 'Mary has a little lamb, little lamb, little lamb.'
        request = Polly.Request(polly_action='SynthesizeSpeech', text=test_text)
        future = client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                self.failIf(future.result() is None, 'nothing is returned')
                break
            print('Waiting for service to be done.')

        res = future.result()

        self.assertIsNotNone(res)
        self.assertTrue(type(res) is Polly.Response)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/pcm', audio_type)
        self.assertTrue(audio_file.endswith('.wav'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*WAVE audio.*', o.decode('utf-8'), flags=re.MULTILINE)
        self.assertIsNotNone(m)

        node.destroy_node()
        rclpy.shutdown()

    def test_plain_text_using_polly_class(self):
        from tts.services.amazonpolly import AmazonPolly
        polly = AmazonPolly()
        test_text = 'Mary has a little lamb, little lamb, little lamb.'
        res = polly.synthesize(text=test_text)
        self.assertIsNotNone(res)
        self.assertTrue(type(res) is Polly.Response)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/pcm', audio_type)
        self.assertTrue(audio_file.endswith('.wav'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*WAVE audio.*', o.decode('utf-8'), flags=re.MULTILINE)
        self.assertIsNotNone(m)

    def test_plain_text_via_synthesizer_node(self):
        rclpy.init()
        node = rclpy.create_node('integtest')
        client = node.create_client(Synthesizer, 'synthesizer')

        retries = 0
        while not client.wait_for_service(timeout_sec=1.0):
            retries += 1
            self.failIf(retries > 3, 'service is not available')

        test_text = 'Mary has a little lamb, little lamb, little lamb.'
        request = Synthesizer.Request(text=test_text)
        future = client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                self.failIf(future.result() is None, 'nothing is returned')
                break
            print('Waiting for service to be done.')

        res = future.result()

        self.assertIsNotNone(res)
        self.assertTrue(type(res) is Synthesizer.Response)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/pcm', audio_type)
        self.assertTrue(audio_file.endswith('.wav'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*WAVE audio.*', o.decode('utf-8'), flags=re.MULTILINE)
        self.assertIsNotNone(m)

        node.destroy_node()
        rclpy.shutdown()

    def test_plain_text_to_mp3_via_polly_node(self):
        rclpy.init()
        node = rclpy.create_node('integtest')
        client = node.create_client(Polly, 'polly')

        retries = 0
        while not client.wait_for_service(timeout_sec=1.0):
            retries += 1
            self.failIf(retries > 3, 'service is not available')

        test_text = 'Mary has a little lamb, little lamb, little lamb.'
        request = Polly.Request(polly_action='SynthesizeSpeech', text=test_text, output_format='mp3')
        future = client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                self.failIf(future.result() is None, 'nothing is returned')
                break
            print('Waiting for service to be done.')

        res = future.result()

        self.assertIsNotNone(res)
        self.assertTrue(type(res) is Polly.Response)


        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/mpeg', audio_type)
        self.assertTrue(audio_file.endswith('.mp3'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*MPEG.*layer III.*', o.decode('utf-8'), flags=re.MULTILINE)
        self.assertIsNotNone(m)

        node.destroy_node()
        rclpy.shutdown()


    def test_simple_ssml_via_polly_node(self):
        rclpy.init()
        node = rclpy.create_node('integtest')
        client = node.create_client(Polly, 'polly')

        retries = 0
        while not client.wait_for_service(timeout_sec=1.0):
            retries += 1
            self.failIf(retries > 3, 'service is not available')

        text = '<speak>Mary has a little lamb, little lamb, little lamb.</speak>'
        request = Polly.Request(polly_action='SynthesizeSpeech', text=text, text_type='ssml')
        future = client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                self.failIf(future.result() is None, 'nothing is returned')
                break
            print('Waiting for service to be done.')

        res = future.result()

        self.assertIsNotNone(res)
        self.assertTrue(type(res) is Polly.Response)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/pcm', audio_type)
        self.assertTrue(audio_file.endswith('.wav'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*WAVE audio.*', o.decode('utf-8'), flags=re.MULTILINE)
        self.assertIsNotNone(m)

        node.destroy_node()
        rclpy.shutdown()


    def test_simple_ssml_via_synthesizer_node(self):
        rclpy.init()
        node = rclpy.create_node('integtest')
        client = node.create_client(Synthesizer, 'synthesizer')

        retries = 0
        while not client.wait_for_service(timeout_sec=1.0):
            retries += 1
            self.failIf(retries > 3, 'service is not available')

        text = '<speak>Mary has a little lamb, little lamb, little lamb.</speak>'
        request = Synthesizer.Request(text=text, metadata='''{"text_type":"ssml"}''')
        future = client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                self.failIf(future.result() is None, 'nothing is returned')
                break
            print('Waiting for service to be done.')

        res = future.result()

        self.assertIsNotNone(res)
        self.assertTrue(type(res) is Synthesizer.Response)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/pcm', audio_type)
        self.assertTrue(audio_file.endswith('.wav'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*WAVE audio.*', o.decode('utf-8'), flags=re.MULTILINE)
        self.assertIsNotNone(m)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
