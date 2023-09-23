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

network DADCA_ACK_n50_u16_a8000m2_3
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
        @display("i=misc/sensor;p=481.32,7959.31");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1061.62,7937.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2198.94,7905.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2622.19,7921.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3098.35,7931.16");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4343.40,7856.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5224.28,7775.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5697.67,7797.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1128.99,7514.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1768.31,7613.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2564.51,7532.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3402.08,7551.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3991.84,7596.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4585.41,7593.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=26.83,7432.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=716.82,7360.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1539.29,7295.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4897.53,7405.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5575.01,7348.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=381.93,7177.54");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2013.35,7221.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2572.84,7056.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3107.29,7115.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4051.13,7055.59");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4500.09,7222.23");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5299.71,7047.75");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5915.43,7104.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=835.35,7003.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1231.51,6952.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2213.03,6859.13");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3523.47,6972.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5043.61,6768.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=80.17,6755.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=556.69,6701.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=976.32,6527.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1699.52,6714.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3961.25,6569.08");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4464.07,6747.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5695.91,6571.47");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=645.30,6354.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1349.72,6416.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1962.85,6353.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2663.96,6465.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3341.26,6467.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4617.12,6355.66");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3073.75,6050.48");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5428.80,6165.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=131.57,5948.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=819.44,5986.08");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1820.76,5901.02");
    }

}

