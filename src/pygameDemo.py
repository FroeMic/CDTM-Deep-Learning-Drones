#! /usr/bin/env python
# -*- coding: utf-8 -*-

import pygame

def land():
    print "land"

def start():
    print "start"

def hover():
    print "Hover"

def forward():
    print "forward"


def backward():
    print "backward"

def left():
    print "left"

def right():
    print "right"

def kill():
    print "goodbye"


def main():
    pygame.init()
    screen = pygame.display.set_mode((640, 320))

    finished = False
    flying = False

    while not finished:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                finished = True
            if event.type == pygame.KEYUP:
                print "KEYUP"
                if event.key == pygame.K_ESCAPE:
                    kill()
                    finished = True
                elif event.key == pygame.K_SPACE:
                    if flying:
                        land()
                        flying = False
                    else:
                        start()
                        flying = True
                elif event.key == pygame.K_w:
                    hover()
                elif event.key == pygame.K_s:
                    hover()
                elif event.key == pygame.K_a:
                    hover()
                elif event.key == pygame.K_d:
                    hover()
            if event.type == pygame.KEYDOWN:
                print "KEYDOWN"
                if event.key == pygame.K_w:
                    forward()
                elif event.key == pygame.K_s:
                    backward()
                elif event.key == pygame.K_a:
                    left()
                elif event.key == pygame.K_d:
                    right()


if __name__ == '__main__':
    main()
