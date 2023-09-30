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

network DADCA_ACK_n50_u3_a8000m2_1
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
        @display("i=misc/sensor;p=516.45,7919.17");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=880.56,7805.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1997.37,7784.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2525.14,7827.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2999.01,7964.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1435.57,7663.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3075.01,7602.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4052.05,7576.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4681.99,7546.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5355.28,7662.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=358.22,7260.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=678.00,7463.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1091.25,7306.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2043.61,7407.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2537.36,7369.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3475.18,7378.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5008.71,7363.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=864.29,7035.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1706.85,7011.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3141.28,7199.53");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5763.03,7222.88");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1295.11,6765.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2238.23,6902.49");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2842.80,6966.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3855.83,6981.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4270.63,6949.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4898.20,6899.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5270.82,6887.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5957.80,6859.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=256.88,6681.45");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=784.28,6519.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2554.60,6747.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3150.68,6657.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3823.14,6551.56");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4397.21,6618.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5672.01,6643.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1804.72,6464.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2224.79,6361.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2641.66,6293.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4110.35,6330.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=275.26,6104.80");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=642.94,6108.96");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1304.07,6261.82");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3006.50,6033.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3485.10,6185.69");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4854.28,6212.52");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5421.76,6226.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5967.11,6248.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1440.24,5896.08");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2011.19,5855.95");
    }

}

