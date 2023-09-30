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

network DADCA_ACK_n50_u4_a8000m2_6
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
        @display("i=misc/sensor;p=258.11,7951.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=818.25,7788.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1716.24,7821.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2731.20,7794.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4351.96,7894.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4995.57,7767.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5359.87,7811.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5964.47,7904.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=251.24,7590.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1399.92,7605.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2272.10,7731.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3760.40,7576.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4561.20,7561.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=36.95,7306.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=666.58,7328.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1039.75,7304.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1988.95,7463.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2419.92,7404.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2851.32,7354.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3390.15,7367.32");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4067.49,7265.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5056.08,7262.11");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5456.45,7405.00");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5844.95,7459.99");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1670.51,7206.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2609.18,7069.72");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3141.17,7062.96");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4467.90,7129.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=137.82,6892.05");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1041.33,6803.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2166.10,6945.22");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3511.45,6926.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4222.62,6785.19");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5659.21,6852.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1730.65,6526.05");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2248.60,6604.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2830.17,6717.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3230.46,6610.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3865.63,6722.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5108.89,6624.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=632.59,6476.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1252.13,6324.97");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2695.11,6365.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3554.02,6378.28");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4736.10,6492.84");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5397.16,6420.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=88.07,6170.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4243.85,6239.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5130.26,6057.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5515.84,6076.54");
    }

}

