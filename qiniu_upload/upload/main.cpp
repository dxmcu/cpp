#include <iostream>
#include "../lib/qiniu/http.h"
#include "../lib/qiniu/rs.h"
#include "../lib/qiniu/io.h"

using namespace std;

bool UploadFileToQiniu(
        const std::string &,
        const std::string &,
        const std::string &,
        const std::string &);

int main()
{
    bool ret = UploadFileToQiniu(
                "/tmp/map.png",
                "media-quixmart-bucket",
                "6IRM4w7Yyl4RfC7l2gPhTd7NbHjD00i3Z4_1wKjT:iLcqNbKDQnmdk6wqyvdxKCdMmVQ=:eyJzY29wZSI6Im1lZGlhLXF1aXhtYXJ0LWJ1Y2tldDpjYWViZDA5Mi05ZTMwLTQ3OTMtOTIxMC0wNGQ5Yzg3ZGYwYzMuIiwiZGVhZGxpbmUiOjE1NTE3NzE3MDB9",
                "caebd092-9e30-4793-9210-04d9c87df0c3."
                );
    cout << "Upload result: " << ret << endl;
    return 0;
}

bool UploadFileToQiniu(
        const std::string &strLocalFile,
        const std::string &strBucket,
        const std::string &strToken,
        const std::string &strFileKey
        )
{
    Qiniu_Global_Init(-1);
    Qiniu_Mac mac;
    mac.accessKey = "6IRM4w7Yyl4RfC7l2gPhTd7NbHjD00i3Z4_1wKjT";
    mac.secretKey = "k6XNa_znUjxPihpTwUtTx0rFR4nJaBMiqBbsGyMp";

    Qiniu_Io_PutRet putRet;
    Qiniu_Client client;
    Qiniu_RS_PutPolicy putPolicy;
    Qiniu_Io_PutExtra putExtra;
    Qiniu_Zero(putPolicy);
    Qiniu_Zero(putExtra);
    putPolicy.scope = strBucket.c_str();
    //char *uptoken = Qiniu_RS_PutPolicy_Token(&putPolicy, &mac);

    //设置机房域名
    //Qiniu_Use_Zone_Beimei(Qiniu_False);
    //Qiniu_Use_Zone_Huabei(Qiniu_True);
    Qiniu_Use_Zone_Huadong(Qiniu_False);
    //Qiniu_Use_Zone_Huanan(Qiniu_True);
    Qiniu_Client_InitMacAuth(&client, 1024, &mac);
    Qiniu_Error error = Qiniu_Io_PutFile(&client,
                                         &putRet,
                                         strToken.c_str(),
                                         strFileKey.c_str(),
                                         strLocalFile.c_str(),
                                         &putExtra);
    //Qiniu_Free(uptoken);
    Qiniu_Client_Cleanup(&client);
    return error.code == 200;
}
