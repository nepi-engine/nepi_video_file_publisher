#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

APP_NAME = 'Vid_File_Publisher' # Use in display menus
FILE_TYPE = 'APP'
APP_DICT = dict(
    description = 'Application for publishing images from video files',
    pkg_name = 'nepi_app_file_pub_vid',
    group_name = 'DATA',
    config_file = 'app_file_pub_vid.yaml',
    app_file = 'file_pub_vid_app_node.py',
    node_name = 'app_file_pub_vid'
)
RUI_DICT = dict(
    rui_menu_name = "Video Publisher", # RUI menu name or "None" if no rui support
    rui_files = ['NepiAppFilePubVid.js'],
    rui_main_file = "NepiAppFilePubVid.js",
    rui_main_class = "FilePubVidApp"
)




