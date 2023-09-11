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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=96.06,23.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=1.28,9.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=44.41,60.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=179.40,13.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=242.63,81.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=315.32,78.31");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=362.91,119.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=140.54,170.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=232.03,200.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=330.48,209.84");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=400.40,252.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=430.71,310.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=510.20,330.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=480.57,160.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=528.76,101.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=469.11,72.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=494.23,227.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=637.79,85.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=597.83,211.93");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=688.20,229.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=686.78,313.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=743.19,367.10");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=694.04,432.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=615.20,358.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=433.42,21.37");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=592.71,20.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=675.95,23.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=751.06,26.98");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=753.34,97.22");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=668.99,138.22");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=589.58,132.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=843.06,32.39");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=898.07,57.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=831.78,130.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=829.58,215.15");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=969.49,61.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=939.63,130.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=743.80,176.72");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=771.91,259.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=804.01,331.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1059.13,140.59");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1049.80,224.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1166.42,176.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1214.43,96.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1279.77,141.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1281.55,43.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1217.59,9.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1103.28,32.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1034.61,13.21");
    }

}

