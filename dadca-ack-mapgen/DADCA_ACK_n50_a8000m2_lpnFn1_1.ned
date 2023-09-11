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

network DADCA_ACK_n50_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=36.18,67.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=40.19,140.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=58.25,236.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=41.38,312.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=147.52,264.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=143.50,324.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=176.71,383.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=212.36,322.26");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=268.66,363.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=129.13,191.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=102.69,411.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=117.08,490.83");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=51.28,452.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=4.44,385.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=185.63,70.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=137.79,8.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=64.62,4.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=214.48,253.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=354.43,288.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=403.03,349.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=424.14,289.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=460.30,210.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=511.39,336.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=481.87,389.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=413.49,436.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=457.76,488.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=566.68,507.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=614.54,421.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=675.60,427.32");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=733.52,407.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=801.40,466.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=790.00,559.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=859.50,514.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=902.85,559.75");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=921.21,482.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1008.08,513.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1062.77,591.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=961.66,427.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1002.16,595.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=948.53,663.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=872.24,663.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1046.83,678.08");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1122.52,687.55");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1185.42,659.87");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1216.92,603.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1126.10,521.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1315.90,589.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1366.21,630.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1392.47,540.66");
    }

}

