include $(shell rospack find mk)/cmake.mk

maps:
	wget http://rnd.yujinrobot.com/pub/maps/lobby.pgm -O resources/maps/lobby.pgm
	wget http://rnd.yujinrobot.com/pub/maps/lab.pgm -O resources/maps/lab.pgm

unison:
	@if [ -f ~/.unison/kobuki.prf ]; then \
                echo ""; \
                echo "Unison profile for kobuki already exists, to overwrite, run:"; \
                echo ""; \
                echo "     make unison-update"; \
                echo ""; \
        else \
                echo ""; \
                echo "Copying unison profile for kobuki to ~/.unison:"; \
                echo ""; \
                mkdir -p ~/.unison; \
                cp resources/unison/kobuki.prf ~/.unison/kobuki.prf; \
                cp resources/unison/yujin.prf ~/.unison/yujin.prf; \
        fi

unison-update:
	@echo ""
	@echo "Updating the kobuki profile with the most recent version."
	@echo ""
	cp resources/unison/kobuki.prf ~/.unison/kobuki.prf
	cp resources/unison/yujin.prf ~/.unison/yujin.prf
	@echo ""

.PHONY: unison unison-update
