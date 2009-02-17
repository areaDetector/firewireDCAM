html/index.html: config.src manual.src
	mkdir -p html
	cd html; dls-make-doxygen-documentation.py $^
	
all install: html/index.html
			
clean uninstall:
	rm -rf html					

	
