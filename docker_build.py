#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import shutil
import getpass


def execute_build(args):
    image = args.image
    docker_file = args.file

    if not os.path.exists(docker_file):
        print('Dockerfile %s not found! Exiting' % docker_file)
        return

    user_name = getpass.getuser()
    print('Saving to image name: ' + image)

    cmd = 'docker build --build-arg USER_NAME=%(user_name)s \
    --build-arg USER_ID=%(user_id)s \
    --build-arg USER_GID=%(group_id)s' \
    %{'user_name': user_name, 'user_id': args.user_id, 'group_id': args.group_id}

    cmd += ' --network=host '
    if args.no_cache:
        cmd += '--no-cache '

    cmd += '-t %s -f %s .' % (image, docker_file)

    print('command = \n\n', cmd)

    if not args.dry_run:
        os.system(cmd)


if __name__ == '__main__':

    default_image_name = "franka-ros-ubuntu-20-user:v0"

    parser = argparse.ArgumentParser()

    parser.add_argument('-f', '--file', type=str, required=True, help='dockerfile to use')
    parser.add_argument('-i', '--image', type=str, default=default_image_name, help='name for new docker image')
    parser.add_argument('--no_cache', action='store_true', help='True if should build without using cache')
    parser.add_argument('-d', '--dry_run', action='store_true', help='True if we should only print the build command without executing')
    parser.add_argument('-uid','--user_id', type=int, help="(optional) user id for this user", default=os.getuid())
    parser.add_argument('-gid','--group_id', type=int, help="(optional) user gid for this user", default=os.getgid())
    args = parser.parse_args()
    execute_build(args)
