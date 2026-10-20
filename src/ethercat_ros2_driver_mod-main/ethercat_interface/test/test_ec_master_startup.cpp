// Offline EtherCAT API doubles: this executable never opens a hardware master.
#include <gtest/gtest.h>
#include "ethercat_interface/ec_master.hpp"
#include "ethercat_interface/ec_slave.hpp"

namespace {
int reference_syncs = 0, slave_syncs = 0, queued_sdos = 0;
uint64_t application_time = 0;
ec_slave_config_t *sdo_target = nullptr;
ec_slave_config_t *reference_target = nullptr;
int reference_selections = 0;
int reference_result = 0;
}
extern "C" {
ec_master_t *ecrt_request_master(unsigned int) { return reinterpret_cast<ec_master_t *>(1); }
ec_domain_t *ecrt_master_create_domain(ec_master_t *) { return reinterpret_cast<ec_domain_t *>(2); }
ec_slave_config_t *ecrt_master_slave_config(ec_master_t *, uint16_t alias, uint16_t position, uint32_t, uint32_t) {
  return reinterpret_cast<ec_slave_config_t *>(100 + alias * 10 + position);
}
int ecrt_master_select_reference_clock(ec_master_t *, ec_slave_config_t *sc) {
  reference_target = sc; ++reference_selections; return reference_result;
}
int ecrt_master_application_time(ec_master_t *, uint64_t t) { application_time = t; return 0; }
int ecrt_slave_config_dc(ec_slave_config_t *, uint16_t, uint32_t, int32_t, uint32_t, int32_t) { return 0; }
int ecrt_master_sync_reference_clock(ec_master_t *) { ++reference_syncs; return 0; }
int ecrt_master_sync_slave_clocks(ec_master_t *) { ++slave_syncs; return 0; }
int ecrt_master_receive(ec_master_t *) { return 0; }
int ecrt_master_send(ec_master_t *) { return 0; }
int ecrt_domain_process(ec_domain_t *) { return 0; }
int ecrt_domain_queue(ec_domain_t *) { return 0; }
int ecrt_domain_state(const ec_domain_t *, ec_domain_state_t *s) { *s = {}; return 0; }
int ecrt_master_state(const ec_master_t *, ec_master_state_t *s) { *s = {}; return 0; }
int ecrt_slave_config_state(const ec_slave_config_t *, ec_slave_config_state_t *s) {
  *s = {}; s->online = 1; s->al_state = EC_AL_STATE_SAFEOP; return 0;
}
int ecrt_slave_config_sdo(ec_slave_config_t *sc, uint16_t index, uint8_t subindex, const uint8_t *data, size_t size) {
  ++queued_sdos; sdo_target = sc;
  EXPECT_EQ(index, 0x6073); EXPECT_EQ(subindex, 0); EXPECT_EQ(size, 2u);
  EXPECT_EQ(EC_READ_U16(data), 3000);
  return 0;
}
}
class FakeSlave : public ethercat_interface::EcSlave {
public:
  FakeSlave() : EcSlave(0x168, 0x11) {}
  int processed = 0;
  int assign_activate_dc_sync() override { return 0x300; }
  void processData(size_t, uint8_t *) override { ++processed; }
};
class TestMaster : public ethercat_interface::EcMaster {
public:
  void attachDomain(FakeSlave &slave) {
    auto *domain = new ethercat_interface::DomainInfo(master_);
    domain_info_[0] = domain;
    ethercat_interface::DomainInfo::Entry entry;
    entry.slave = &slave; entry.num_pdos = 1;
    entry.offset = new uint32_t[1]{0}; entry.bit_position = new uint32_t[1]{0};
    domain->entries.push_back(entry);
    domain->domain_pd = bytes;
  }
  void operational() { allSlavesInOP_ = true; }
private:
  uint8_t bytes[8]{};
};
TEST(MasterStartup, SynchronizesBeforeOpButKeepsCommandsGated) {
  reference_syncs = slave_syncs = 0;
  TestMaster master; FakeSlave slave;
  master.setCtrlFrequency(500);
  master.addSlave(6, 0, &slave);
  const auto initial_time = application_time;
  master.attachDomain(slave);
  master.update();
  master.writeData();
  EXPECT_EQ(reference_syncs, 2); EXPECT_EQ(slave_syncs, 2);
  EXPECT_EQ(slave.processed, 0);
  EXPECT_GE(application_time, initial_time);
  EXPECT_LT(application_time - initial_time, 1000000000ULL);
  master.operational(); master.writeData();
  EXPECT_EQ(slave.processed, 1);
}
TEST(MasterStartup, StartupSdoUsesAliasConfiguration) {
  queued_sdos = 0;
  TestMaster master; FakeSlave x, y;
  master.setCtrlFrequency(500);
  master.addSlave(6, 0, &x); master.addSlave(7, 0, &y);
  ethercat_interface::SdoConfigEntry entry;
  entry.index = 0x6073; entry.sub_index = 0; entry.data_type = "uint16"; entry.data = 3000;
  EXPECT_EQ(master.configSlaveSdo(7, 0, entry), 0);
  EXPECT_EQ(sdo_target, reinterpret_cast<ec_slave_config_t *>(170));
  EXPECT_EQ(master.configSlaveSdo(6, 0, entry), 0);
  EXPECT_EQ(sdo_target, reinterpret_cast<ec_slave_config_t *>(160));
  EXPECT_NE(master.configSlaveSdo(9, 0, entry), 0);
  EXPECT_EQ(queued_sdos, 2);
}

TEST(MasterStartup, FirstConfiguredDcSlaveIsReference) {
  reference_selections = 0; reference_target = nullptr;
  TestMaster master;
  ethercat_interface::EcSlave gateway(0xdf0, 0x14);
  FakeSlave x, y;
  master.setCtrlFrequency(500);
  master.addSlave(0, 3, &gateway);
  EXPECT_EQ(reference_selections, 0);
  master.addSlave(6, 0, &x);
  master.addSlave(7, 0, &y);
  EXPECT_EQ(reference_selections, 1);
  EXPECT_EQ(reference_target, reinterpret_cast<ec_slave_config_t *>(160));
}
TEST(MasterStartup, ReferenceSelectionFailureStopsConfiguration) {
  TestMaster master; FakeSlave x;
  master.setCtrlFrequency(500);
  reference_result = -1;
  EXPECT_THROW(master.addSlave(6, 0, &x), std::runtime_error);
  reference_result = 0;
}
