import requests
import threading

class download:
    def download_novel(self, url, statistics_callback):
        print(f'thread : {threading.get_ident()} start download now : {url}')
        response = requests.get(url)
        response.encoding = 'utf-8'
        statistics_callback(url, response.text)

    def start_download(self, url, statistics_callback):
        thread = threading.Thread(target = self.download_novel, args = (url, statistics_callback))
        thread.start()

def count_text_size(url, result):
    print(f"{url}:{len(result)}->{result[:5]}")

def main():
    download = download()
    download.start_download("http://localhost:8000/core/node1/node1/novel/chapter1.txt",count_text_size)
    download.start_download("http://localhost:8000/core/node1/node1/novel/chapter2.txt",count_text_size)
    download.start_download("http://localhost:8000/core/node1/node1/novel/chapter3.txt",count_text_size)