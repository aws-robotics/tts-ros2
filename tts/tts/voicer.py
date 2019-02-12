# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from tts_interfaces.srv import Synthesizer

import rclpy
from rclpy.node import Node
import sys
import json
import os
import time

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Voicer(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.synthesizer = self.create_client(Synthesizer, 'synthesizer')
        self.logger = self.get_logger()
        Gst.init(None)

    def handle_result(self, synthesizer_response):
        result = synthesizer_response.result
        try:
            r = json.loads(result)
        except Exception as e:
            s = 'Expecting JSON from synthesizer but got {}'.format(result)
            self.logger.error('{}. Exception: {}'.format(s, e))
            return

        result = ''

        if 'Audio File' in r:
            audio_file = r['Audio File']
            self.logger.info('Will play {}'.format(audio_file))
            self.play(audio_file)
            result = audio_file

        if 'Exception' in r:
            result = '[ERROR] {}'.format(r)
            self.logger.error(result)

        return result

    def play(self, filename):
        self.logger.info('using gstreamer to play the audio')

        playbin = Gst.ElementFactory.make('playbin', 'player')

        bus = playbin.get_bus()

        playbin.props.uri = 'file://' + os.path.abspath(filename)
        time.sleep(0.5)  # sometimes gst needs time to get ready for unknown reasons
        set_result = playbin.set_state(Gst.State.PLAYING)
        if set_result != Gst.StateChangeReturn.ASYNC:
            raise RuntimeError("gstreamer error: playbin.set_state returned " + repr(set_result))

        bus.poll(Gst.MessageType.EOS, Gst.CLOCK_TIME_NONE)
        playbin.set_state(Gst.State.NULL)

    def speak(self):
        req = Synthesizer.Request()

        req.text = sys.argv[1] if len(sys.argv) > 1 else 'I got no idea.'
        req.metadata = sys.argv[2] if len(sys.argv) > 2 else ''

        while not self.synthesizer.wait_for_service(timeout_sec=1.0):
            self.logger.warn('service not available, waiting again...')

        future = self.synthesizer.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)

            if future.done():
                if future.result() is not None:
                    self.logger.info('Result: %s' % str(future.result()))
                    self.handle_result(future.result())
                else:
                    self.logger.error('Exception while calling service: %r' % future.exception())
                break

        self.logger.info('Done speaking {}'.format(req.text))


def main():
    rclpy.init()
    node = Voicer()
    node.speak()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
