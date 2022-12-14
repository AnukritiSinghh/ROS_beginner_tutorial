cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/") --output-file=results/cppcheck_process.txt > results/cppcheck_result.txt
echo "Done Processing. Results are stored in results/cppcheck_process.txt, results/cppcheck_result.txt"
