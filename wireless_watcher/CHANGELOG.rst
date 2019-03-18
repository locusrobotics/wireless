^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wireless_watcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2019-03-18)
------------------

0.1.0 (2019-01-16)
------------------
* Add ethtool dependency
* scale link_quality not raw
* scaling frequency and link quality for mikrotik
* removing surplus hash
* correcting roslint errors
* correcting CMake
* removing extra newline
* adding roslaunch test and roslint test
* cleaning up, especially mikrotik api
* clarifying api
* correcting default
* cleaning up mikrotik watcher
* re-ordering imports
* roboy_name->robot_name
* cleaning up mikrotik api
* adding locus BSD, cleaning up mikrotik watcher
* cleaning up mikrotik api, adding bsd, removing duplicate code
* adding locus BSD to mikrotik_watcher_node
* adding myself as maintainer
* removing kernel param, kernel assigned by detection only
* updating launch file
* robot names has underscore
* removing requirements
* removing firmware as arg
* correct_spaces
* removing the firmware version param as it is now detected
* correcting firmware lookup
* reading os, not setting manually
* changing default namespace to be robot namespace
* correcting qualty reporting
* correcting import error, correcting quality reporting, correcting name lookup
* more convenient launch file
* reordering cmake
* adding launch file for mikrotik
* adding kernel reporting
* correcting link_quality
* adding mikrotik watcher, updating watcher with new message contents
* moving nodes to scripts, adding python setup
* Package format 2.
* Further pep8 fixups.
* Auto-detect wl* device if not passed explicitly.
* Contributors: David Elkan, Mike Purvis, Paul Bovbel

0.0.7 (2015-09-09)
------------------
* Added frequency to watcher node
* Contributors: Alex Bencz

0.0.6 (2015-09-02)
------------------
* Added queue_size parameter to publishers
* Contributors: Mustafa Safri

0.0.5 (2015-08-25)
------------------
* Added checks for missing fields
* Contributors: Mustafa Safri

0.0.4 (2015-06-25)
------------------
* Add install rule for launch dir
* Contributors: Paul Bovbel

0.0.3 (2015-06-17)
------------------
* Added ESSID and BSSID fields, use floats for bitrate
* Contributors: Alex Bencz

0.0.2 (2013-10-24)
------------------
* Workaround to suppress DummyThread error spew.
* Add simple boolean to publish ethernet-has-ip status.

0.0.1 (2013-10-17)
------------------
* Catkinize wireless_watcher
