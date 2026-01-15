import os
import pandas as pd

class BasicTransfer:
    def __init__(self, args):
        self.args = args

    def get_all_data(self) -> list:
        file_names = os.listdir(self.args.data_folder)
        data_list = []
        for file_name in file_names:
            data_list.append(os.path.join(self.args.data_folder, file_name))
        return data_list

    def _process_data(self, file_path: str) -> pd.DataFrame:
        raise NotImplementedError

    def _save_data(self, processed_data: pd.DataFrame, file_name: str) -> None:
        file_name = file_name.split("/")[-1]
        file_name = file_name.split(".")[0]
        save_path = os.path.join(self.args.save_folder, file_name+".csv")
        processed_data.to_csv(save_path)

    def run(self) -> None:
        data_list = self.get_all_data()
        for file_path in data_list:
            processed_data = self._process_data(file_path)
            self._save_data(processed_data, file_path)


class HighDTransfer(BasicTransfer):

    def __init__(self, args):
        super(HighDTransfer, self).__init__(args)

    def _process_data(self, file_path: str) -> pd.DataFrame:
        # 对于不同的数据集transfer，补充这个函数即可。返回处理好的dataframe。
        # 一般情况保持 basictransfer 不动
        pass


class InDTransfer(BasicTransfer):
    def __init__(self, args):
        super(InDTransfer, self).__init__(args)

    def _process_data(self, file_path: str) -> pd.DataFrame:
        pass

