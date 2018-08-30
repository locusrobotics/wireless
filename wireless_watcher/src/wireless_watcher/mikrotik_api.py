#!/usr/bin/python

import sys, posix, time, md5, binascii, socket, select


class RouterOSApi(object):
    "Routeros api"
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
        r = []
        while 1:
            i = self.readSentence(verbose)
            if len(i) == 0:
                continue
            reply = i[0]
            attrs = {}
            for w in i[1:]:
                j = w.find('=', 1)
                if (j == -1):
                    attrs[w] = ''
                else:
                    attrs[w[:j]] = w[j+1:]
            r.append((reply, attrs))
            if reply == '!done':
                return r

    def writeSentence(self, words, verbose):
        ret = 0
        for w in words:
            self.writeWord(w, verbose)
            ret += 1
        self.writeWord('', verbose)
        return ret

    def readSentence(self, verbose=True):
        r = []
        while 1:
            w = self.readWord(verbose)
            if w == '':
                return r
            r.append(w)

    def writeWord(self, w, verbose=True):
        if verbose:
            print "<<< " + w
        self.writeLen(len(w))
        self.writeStr(w)

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
            r = self.sk.send(text[n:])
            if r == 0:
                raise RuntimeError, "connection closed by remote end"
            n += r

    def readStr(self, length):
        ret = ''
        while len(ret) < length:
            s = self.sk.recv(length - len(ret))
            if s == '':
                raise RuntimeError, "connection closed by remote end"
            ret += s
        return ret
    def silentGet(self, command_list):
        return self.talk(command_list, verbose=False)
    
    def getAllInterfaces(self):
        return self.talk(["/interface/getall"], verbose=False)
    
    def getWirelessInterfaces(self):
        return self.silentGet(["/interface/print", "?type=wlan"])
    
    def getEthernetInterfaces(self):
        return self.silentGet(["/interface/print", "?type=ether"])
    
    def monitorWirelessInterface(self, interface="wlan1"):
        return self.silentGet(["/interface/wireless/monitor","=numbers={}".format(interface),"=once="])

def main():
    apiros = RouterOSApi(sys.argv[1], 8728);

    # use default username and pasword if not specified
    if len(sys.argv) == 4:
      if not apiros.login(sys.argv[2], sys.argv[3], verbose=False):
        return
    elif len(sys.argv) == 3:
      if not apiros.login(sys.argv[2], "", verbose=False):
        return
    else :
      if not apiros.login("admin", "", verbose=False):
        return

    inputsentence = []
    detected_interfaces = [interface[1] for interface in apiros.getWirelessInterfaces() if interface[0] == "!re"]
    if len(detected_interfaces) == 1:
        name = detected_interfaces[0]["=name"]
        print name
    
    res =  []
        


if __name__ == '__main__':
  if len(sys.argv) == 1:
    print "Usage: %s IP [user] [pass]" % str(sys.argv[0])
  else:
    main()
