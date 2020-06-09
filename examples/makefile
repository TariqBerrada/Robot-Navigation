IMAGES_DIR := $(dir $(abspath $(dir $$PWD)))resources/world_visualizations

example_world1:
	@python example_world1.py
	@ffmpeg -framerate 24 -i $(IMAGES_DIR)/world1/world1_%06d.jpg $(IMAGES_DIR)/world1/film.mp4
	@vlc $(IMAGES_DIR)/world1/film.mp4