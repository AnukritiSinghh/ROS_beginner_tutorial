cpplint $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/") $( find . -name *.hpp | grep -vE -e "^./build/" -e "^./vendor/") > results/cpplint_result.txt
echo "Done Processing. Results are stored in results/cpplint_result.txt"
