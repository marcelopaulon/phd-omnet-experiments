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

network DADCA_ACK_n50_u3_a8000m2_0
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
        @display("i=misc/sensor;p=682.75,7788.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2186.24,7886.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2797.69,7843.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3405.73,7948.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4364.52,7950.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4879.70,7790.34");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5364.00,7805.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1222.55,7704.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1651.14,7734.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3131.93,7600.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3788.60,7559.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=944.36,7449.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1988.37,7502.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2576.26,7350.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3223.63,7259.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4352.31,7303.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5137.85,7438.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=108.27,7234.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=793.92,7133.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1462.97,7191.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1824.96,7099.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2175.00,7057.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=257.58,6854.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1564.51,6822.95");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2976.44,6865.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4107.76,6833.28");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4538.65,6774.24");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5051.90,6763.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5687.47,6806.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=791.85,6701.42");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1178.96,6733.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2456.60,6616.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3765.96,6650.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=143.89,6485.57");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1420.06,6432.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2083.26,6484.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3063.04,6373.21");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3712.00,6299.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4374.10,6463.57");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5293.18,6453.62");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5828.41,6391.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=442.27,6029.95");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=899.75,6136.30");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1377.06,6028.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2757.70,6090.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3439.18,6039.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4010.30,6047.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4696.10,6240.10");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5995.84,6049.59");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2321.54,5968.77");
    }

}

