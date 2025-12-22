"""
Quick validation script for the ChatKit RAG integration.
This script validates that all required files and structures are in place.
"""
import os
import sys
from pathlib import Path


def validate_implementation():
    """
    Validate that all components of the ChatKit RAG integration are properly implemented.
    """
    print("Starting validation of ChatKit RAG integration...")
    
    # Define the expected file structure
    backend_path = Path("Backend")
    
    # Check that the backend directory exists
    if not backend_path.exists():
        print("❌ Backend directory does not exist")
        return False
    
    print("✅ Backend directory exists")
    
    # Check for required directories
    required_dirs = [
        "src/chat",
        "src/rag", 
        "src/core",
        "src/services",
        "tests/unit",
        "tests/integration", 
        "tests/contract",
        "docs"
    ]
    
    all_dirs_exist = True
    for dir_path in required_dirs:
        full_path = backend_path / dir_path
        if not full_path.exists():
            print(f"❌ Required directory does not exist: {dir_path}")
            all_dirs_exist = False
        else:
            print(f"✅ Directory exists: {dir_path}")
    
    if not all_dirs_exist:
        return False
    
    # Check for required files
    required_files = [
        "src/chat/models.py",
        "src/chat/services.py", 
        "src/chat/endpoints.py",
        "src/rag/models.py",
        "src/rag/services.py",
        "src/rag/agents.py", 
        "src/rag/utils.py",
        "src/core/config.py",
        "src/core/database.py",
        "src/core/vector_db.py", 
        "src/core/exceptions.py",
        "src/core/openai_client.py",
        "src/core/constants.py",
        "src/core/rate_limiter.py",
        "src/services/session_service.py",
        "src/services/validation_service.py",
        "src/main.py",
        "requirements.txt",
        "pyproject.toml",
        "docs/api.md",
        "docs/development.md",
        "tests/unit/test_session_service.py",
        "tests/unit/test_validation_service.py",
        "tests/contract/test_chat_completion.py",
        "tests/contract/test_chat_validate.py",
        "tests/integration/test_chat_flow.py",
        "tests/integration/test_response_accuracy.py",
        "tests/integration/test_conversation_flow.py"
    ]
    
    all_files_exist = True
    for file_path in required_files:
        full_path = backend_path / file_path
        if not full_path.exists():
            print(f"❌ Required file does not exist: {file_path}")
            all_files_exist = False
        else:
            print(f"✅ File exists: {file_path}")
    
    if not all_files_exist:
        return False
    
    # Check for the feature spec files
    spec_path = Path("specs/005-chatkit-rag-integration")
    spec_files = [
        "spec.md",
        "plan.md",
        "data-model.md", 
        "tasks.md",
        "quickstart.md",
        "research.md",
        "checklists/requirements.md",
        "contracts/openapi.yaml"
    ]
    
    for file_path in spec_files:
        full_path = spec_path / file_path
        if not full_path.exists():
            print(f"⚠️  Spec file does not exist: {file_path} (This may be OK depending on project structure)")
        else:
            print(f"✅ Spec file exists: {file_path}")
    
    print("\n🎉 All structural validation checks passed!")
    print("\nChatKit RAG Integration project structure is properly implemented.")
    print("\nTo start the service, run:")
    print("   cd Backend")
    print("   uvicorn src.main:app --reload --host 0.0.0.0 --port 8000")
    
    return True


if __name__ == "__main__":
    success = validate_implementation()
    if not success:
        print("\n❌ Validation failed. Please check the above errors.")
        sys.exit(1)
    else:
        print("\n✅ Structural validation completed successfully!")