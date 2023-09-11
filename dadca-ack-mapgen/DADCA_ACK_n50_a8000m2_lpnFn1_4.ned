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

network DADCA_ACK_n50_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=47.73,162.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=81.81,41.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=211.54,72.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=167.75,15.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=222.19,165.13");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=242.18,256.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=144.91,271.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=123.96,343.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=196.86,342.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=227.38,395.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=53.98,461.68");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=60.79,534.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=153.24,466.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=232.70,466.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=230.84,582.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=283.53,653.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=277.93,514.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=295.11,395.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=368.68,384.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=406.38,328.76");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=372.62,260.46");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=131.79,191.52");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=288.54,100.74");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=386.82,86.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=359.92,164.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=273.90,316.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=329.18,455.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=438.18,453.06");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=526.52,442.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=599.86,481.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=563.11,564.90");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=566.65,657.44");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=662.58,630.49");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=630.65,702.56");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=630.70,573.08");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=707.74,556.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=696.31,486.43");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=662.75,403.93");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=731.40,366.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=729.38,288.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=802.13,318.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=787.55,395.15");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=833.37,457.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=756.18,469.97");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=909.96,452.09");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=870.56,391.40");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=924.45,322.97");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=917.91,256.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=884.71,174.63");
    }

}

