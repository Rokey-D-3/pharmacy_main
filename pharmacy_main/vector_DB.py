# Vector DB 구축용 코드
from langchain_community.vectorstores import Chroma
from langchain_openai import OpenAIEmbeddings
from langchain_community.document_loaders import TextLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
import os
from dotenv import load_dotenv
load_dotenv()

# 현재 실행 중인 파일 경로 기준 디렉토리 구하기
current_dir = os.path.dirname(os.path.abspath(__file__))

# OpenAI API Key
openai_api_key = os.getenv("/home/park/ros2_ws/src/pharmacy_main/resource/.env")

# 1️⃣ Indications 텍스트 로드
# → 현재 파일 위치 기준 drug_symptom_mapping.txt 사용
# input_file_path = os.path.join(current_dir,"/resource", "drug_symptom_mapping.txt")
input_file_path = '/home/park/ros2_ws/src/pharmacy_main/resource/drug_symptom_mapping.txt'
loader = TextLoader(input_file_path)
documents = loader.load()

# 2️⃣ 텍스트 chunking
# → chunk_size 500 token 단위로 잘라서 Vector DB 검색 정확도 높임
text_splitter = RecursiveCharacterTextSplitter(
    chunk_size=500,
    chunk_overlap=100
)
docs = text_splitter.split_documents(documents)

# 3️⃣ Embedding 모델 설정
embeddings = OpenAIEmbeddings(openai_api_key=openai_api_key)

# 4️⃣ Vector DB 구축
output_db_path = os.path.join(current_dir, "chroma_db")
db = Chroma.from_documents(
    docs,
    embeddings,
    persist_directory=output_db_path  # 저장 경로 (현재 디렉토리 기준)
)

# 5️⃣ Vector DB 저장 완료
db.persist()

print(f"✅ Vector DB 구축 완료 → {output_db_path} 폴더에 저장됨")
