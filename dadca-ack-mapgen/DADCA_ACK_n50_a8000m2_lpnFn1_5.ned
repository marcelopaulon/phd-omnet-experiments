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

network DADCA_ACK_n50_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=94.18,166.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=151.63,140.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=40.81,228.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=15.61,314.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=72.73,336.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=46.16,413.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=122.48,449.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=150.74,347.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=225.46,314.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=220.58,127.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=214.74,63.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=134.84,41.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=52.85,11.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=188.69,6.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=324.00,93.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=274.76,194.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=309.67,270.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=221.65,239.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=114.87,278.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=377.70,125.37");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=335.88,184.38");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=377.26,252.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=197.57,403.59");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=245.81,443.36");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=289.83,511.01");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=386.59,526.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=440.81,578.66");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=504.08,557.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=599.09,552.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=548.40,604.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=440.99,500.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=420.43,432.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=417.95,349.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=486.15,234.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=432.98,199.24");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=332.19,3.19");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=189.37,509.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=117.37,555.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=152.64,610.08");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=147.51,702.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=243.21,690.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=255.00,605.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=362.83,710.63");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=354.74,789.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=272.83,772.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=253.55,830.75");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=193.31,813.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=112.26,782.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=118.62,917.99");
    }

}

