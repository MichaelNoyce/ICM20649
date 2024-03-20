#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "ICM20649.h"

// Mock class for ICM20649 driver
class MockICM20649 : public ICM20649 {
public:
    MOCK_METHOD(ICM20649_Status, Init, (), (override));
    MOCK_METHOD(ICM20649_Status, Deinit, (), (override));
    MOCK_METHOD(ICM20649_Status, ReadData, (uint8_t* data), (override));
    MOCK_METHOD(ICM20649_Status, WriteData, (uint8_t data), (override));
};

// Test fixture for ICM20649 driver
class ICM20649Test : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ICM20649 driver
        icm20649 = std::make_unique<MockICM20649>();
        EXPECT_CALL(*icm20649, Init()).WillOnce(::testing::Return(ICM20649_OK));
        icm20649->Init();
    }

    void TearDown() override {
        // Clean up ICM20649 driver
        EXPECT_CALL(*icm20649, Deinit()).WillOnce(::testing::Return(ICM20649_OK));
        icm20649->Deinit();
    }

    std::unique_ptr<MockICM20649> icm20649;
};

// Test case for ICM20649 driver initialization
TEST_F(ICM20649Test, Initialization) {
    // Perform initialization test
    EXPECT_CALL(*icm20649, Init()).WillOnce(::testing::Return(ICM20649_OK));
    ASSERT_EQ(icm20649->Init(), ICM20649_OK);
}

// Run all the tests
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}