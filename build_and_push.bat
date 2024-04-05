@echo off
:: 빌드 커맨드를 실행하고 결과를 체크
call mingw32-make.exe -j4 C:\KC\build-untitled3-mingw-Debug || goto :error

:: 빌드가 성공적으로 완료되었으면 git 명령 실행
git add .
git update-index --assume-unchanged untitled3.pro.user
git commit -m "Successful build"
git push
goto :end

:error
echo Build failed, not committing or pushing.
:end