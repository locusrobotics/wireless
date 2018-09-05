# Software License Agreement (BSD License)
#
#  Copyright (c) 2018, Locus Robotics
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  # Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  # Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  # Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import binascii
import md5
import posix
import time
import socket
import select
import sys


class RouterOSApi(object):
    '''Provides API access to mikrotik wifi devices
    '''
    def __init__(self, hostname, port=8728):
        self.sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sk.connect((hostname, port))
        self.currenttag = 0

    def login(self, username="admin", pwd="", verbose=True):
        for repl, attrs in self.talk(["/login", "=name=" + username, "=password=" + pwd], verbose):
            if repl == '!trap':
                raise ValueError("Failed to login, check username and password and try again.")
            elif '=ret' in attrs.keys():
                #for repl, attrs in self.talk(["/login"]):
                chal = binascii.unhexlify(attrs['=ret'])
                md = md5.new()
                md.update('\x00')
                md.update(pwd)
                md.update(chal)
                for repl2, attrs2 in self.talk(["/login", "=name=" + username,
                                               "=response=00" + binascii.hexlify(md.digest())], verbose):
                    if repl2 == '!trap':
                        return False
        return True

    def talk(self, words, verbose=True):
        if self.writeSentence(words, verbose) == 0:
            return
        res = []
        while 1:
            sentence = self.readSentence(verbose)
            if len(sentence) == 0:
                continue
            reply = sentence[0]
            attrs = {}
            for word in sentence[1:]:
                j = word.find('=', 1)
                if (j == -1):
                    attrs[word] = ''
                else:
                    attrs[word[:j]] = word[j+1:]
            res.append((reply, attrs))
            if reply == '!done':
                return res

    def writeSentence(self, words, verbose):
        ret = 0
        for word in words:
            self.writeWord(word, verbose)
            ret += 1
        self.writeWord('', verbose)
        return ret

    def readSentence(self, verbose=True):
        res = []
        while 1:
            word = self.readWord(verbose)
            if word == '':
                return res
            res.append(word)

    def writeWord(self, word, verbose=True):
        if verbose:
            print "<<< " + word
        self.writeLen(len(word))
        self.writeStr(word)

    def readWord(self, verbose=True):
        ret = self.readStr(self.readLen())
        if verbose:
            print ">>> " + ret
        return ret

    def writeLen(self, l):
        if l < 0x80:
            self.writeStr(chr(l))
        elif l < 0x4000:
            l |= 0x8000
            self.writeStr(chr((l >> 8) & 0xFF))
            self.writeStr(chr(l & 0xFF))
        elif l < 0x200000:
            l |= 0xC00000
            self.writeStr(chr((l >> 16) & 0xFF))
            self.writeStr(chr((l >> 8) & 0xFF))
            self.writeStr(chr(l & 0xFF))
        elif l < 0x10000000:
            l |= 0xE0000000
            self.writeStr(chr((l >> 24) & 0xFF))
            self.writeStr(chr((l >> 16) & 0xFF))
            self.writeStr(chr((l >> 8) & 0xFF))
            self.writeStr(chr(l & 0xFF))
        else:
            self.writeStr(chr(0xF0))
            self.writeStr(chr((l >> 24) & 0xFF))
            self.writeStr(chr((l >> 16) & 0xFF))
            self.writeStr(chr((l >> 8) & 0xFF))
            self.writeStr(chr(l & 0xFF))

    def readLen(self):
        c = ord(self.readStr(1))
        if (c & 0x80) == 0x00:
            pass
        elif (c & 0xC0) == 0x80:
            c &= ~0xC0
            c <<= 8
            c += ord(self.readStr(1))
        elif (c & 0xE0) == 0xC0:
            c &= ~0xE0
            c <<= 8
            c += ord(self.readStr(1))
            c <<= 8
            c += ord(self.readStr(1))
        elif (c & 0xF0) == 0xE0:
            c &= ~0xF0
            c <<= 8
            c += ord(self.readStr(1))
            c <<= 8
            c += ord(self.readStr(1))
            c <<= 8
            c += ord(self.readStr(1))
        elif (c & 0xF8) == 0xF0:
            c = ord(self.readStr(1))
            c <<= 8
            c += ord(self.readStr(1))
            c <<= 8
            c += ord(self.readStr(1))
            c <<= 8
            c += ord(self.readStr(1))
        return c

    def writeStr(self, text):
        n = 0;
        while n < len(text):
            res = self.sk.send(text[n:])
            if res == 0:
                raise RuntimeError("Connection closed by remote end")
            n += res

    def readStr(self, length):
        ret = ''
        while len(ret) < length:
            string = self.sk.recv(length - len(ret))
            if string == '':
                raise RuntimeError("Connection closed by remote end")
            ret += string
        return ret


def main():
    apiros = RouterOSApi(sys.argv[1], 8728);
    # use default username and pasword if not specified
    if len(sys.argv) == 4:
      if not apiros.login(sys.argv[2], sys.argv[3]):
        return
    elif len(sys.argv) == 3:
      if not apiros.login(sys.argv[2], ""):
        return
    else:
      if not apiros.login("admin", ""):
        return

    inputsentence = []

    while 1:
        res = select.select([s, sys.stdin], [], [], None)
        if s in res[0]:
            # something to read in socket, read sentence
            sentence = apiros.readSentence()

        if sys.stdin in res[0]:
            # read line from input and strip off newline
            line = sys.stdin.readline()
            line = line[:-1]

            # if empty line, send sentence and start with new
            # otherwise append to input sentence
            if line == '':
                apiros.writeSentence(inputsentence)
                inputsentence = []
            else:
                inputsentence.append(line)


if __name__ == '__main__':
  if len(sys.argv) == 1:
    print("Usage: {} IP [user] [pass]".format(sys.argv[0]))
  else:
    main()
