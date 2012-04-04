include $(shell rospack find ycs_build)/mk/ycsinstall.mk

unison:
	@if [ -f ~/.unison/kobukibot.prf ]; then \
                echo ""; \
                echo "Unison profile for kobukibot already exists, to overwrite, run:"; \
                echo ""; \
                echo "     make unison-update"; \
                echo ""; \
        else \
                echo ""; \
                echo "Copying unison profile for kobukibot to ~/.unison:"; \
                echo ""; \
                mkdir -p ~/.unison; \
                cp resources/unison/kobukibot.prf ~/.unison/kobukibot.prf; \
        fi

unison-update:
	@echo ""
	@echo "Updating the kobukibot profile with the most recent version."
	@echo ""
	cp resources/unison/kobukibot.prf ~/.unison/kobukibot.prf
	@echo ""

.PHONY: unison unison-update
