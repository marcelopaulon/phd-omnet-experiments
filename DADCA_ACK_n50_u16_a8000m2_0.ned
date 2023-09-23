package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_u16_a8000m2_0
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=437.30,7793.29");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2330.15,7911.35");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2686.37,7801.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3335.02,7955.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3998.88,7770.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=973.21,7654.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1587.81,7573.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2133.69,7597.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4652.36,7665.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5597.45,7737.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5953.23,7532.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=248.20,7277.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=552.31,7458.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2371.70,7278.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2887.41,7457.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3248.95,7495.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4145.87,7409.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5263.35,7440.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=593.35,7066.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1011.05,7186.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1373.54,7030.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1731.67,7180.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3722.69,7232.74");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4776.31,7200.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5705.98,7233.50");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=951.56,6797.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2843.63,6940.96");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3259.04,6979.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4069.83,6962.89");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4448.35,7009.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=55.17,6713.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=548.67,6701.55");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1511.89,6537.58");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2063.93,6635.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3230.95,6554.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3744.04,6710.46");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4351.34,6625.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4908.04,6731.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5257.57,6612.24");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5843.93,6662.18");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=87.86,6329.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1116.11,6386.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2517.60,6394.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3794.78,6352.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4895.76,6314.07");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=541.60,6043.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1683.46,6050.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2959.93,6148.01");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3314.03,6086.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4393.01,6262.68");
    }

}

