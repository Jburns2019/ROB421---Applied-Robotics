.DEFAULT_GOAL := build_website
main_page := index.html
blockly_page := .\Misty-Blockly\misty_blockly.html
python_page := .\Misty-Python\misty_python.html
assembly_page := .\Pupper-Assembly\pupper_assembly.html
racing_page := .\Pupper-Racing\pupper_racing.html
eot_page := .\Pupper-EOT-Project\pupper_eot_project.html

#For John.
build_website:
	git add ${main_page}
	git add ${blockly_page} ${python_page}
	git add ${assembly_page} ${racing_page} ${eot_page}
	git commit -m "Update website."
	git pull
	git push