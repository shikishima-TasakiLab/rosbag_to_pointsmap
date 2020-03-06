#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <iostream>

#define NO_FILE_DIR_EXIST 0
#define FILE_EXIST 1
#define DIR_EXIST 2

int filedirE(std::string path);
void makeDirectory(std::string dir_path);
std::string get_extension(std::string file_path);

/* ファイル・ディレクトリの存在確認を行う
std::string path : ファイル・ディレクトリのパス
・NO_FILE_DIR_EXIST(0):
指定したパスのファイル・ディレクトリは存在しない 
・FILE_EXIST(1):
指定したパスのファイルが存在する
・DIR_EXIST(2):
指定したパスのディレクトリが存在する */
int filedirE(std::string path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0){
        return NO_FILE_DIR_EXIST;
    }
    else{
        mode_t m = st.st_mode;
        if(S_ISDIR(m)){
            return DIR_EXIST;
        }
        else{
            return FILE_EXIST;
        }
    }
}

/* ディレクトリを作成する
std::string dir_path : 作成するディレクトリのパス
 */
void makeDirectory(std::string dir_path)
{
    if (filedirE(dir_path) != DIR_EXIST){
        if (mkdir(dir_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH) == 0){
            std::cout << dir_path << " を作成しました" << std::endl;
        }
        else{
            std::cerr << dir_path << " の作成に失敗しました" << std::endl;
        }
    }
    else{
        std::cerr << dir_path << " は既に存在します" << std::endl;
    }
}

/* 拡張子を取得する
std::string file_path : 拡張子を取得するファイルのパス
return : std::string 拡張子
*/
std::string get_extension(std::string file_path)
{
    int ext_i = file_path.find_last_of(".");
    return file_path.substr(ext_i, file_path.size() - ext_i);
}
