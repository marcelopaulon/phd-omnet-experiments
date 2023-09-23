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

network DADCA_ACK_n50_u4_a8000m2_7
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
        @display("i=misc/sensor;p=1467.13,7924.55");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2407.57,7872.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2800.95,7992.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3771.05,7966.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5877.16,7793.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=89.92,7519.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=438.36,7617.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=764.85,7744.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1165.11,7654.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1531.45,7515.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2644.48,7556.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3976.45,7610.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4677.44,7651.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=696.49,7348.42");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2071.14,7381.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3607.95,7383.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4493.94,7295.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5146.99,7462.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5992.91,7297.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=87.60,7011.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1165.79,7103.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3058.68,7243.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4040.41,7210.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5428.58,7155.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=650.42,6968.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1786.52,6891.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2399.23,6921.72");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2853.42,6934.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3373.60,6949.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4369.22,6889.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4757.22,6971.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5111.00,6928.51");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=629.64,6539.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1200.81,6572.35");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2136.95,6605.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3844.92,6758.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5510.65,6668.21");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=224.95,6417.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=965.63,6293.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2661.01,6511.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3153.48,6356.09");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4210.47,6497.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4586.73,6512.43");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=146.31,6035.34");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=482.49,6144.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1245.54,6073.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1646.78,6247.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2142.66,6219.04");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3444.23,6087.83");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4141.44,6144.62");
    }

}

