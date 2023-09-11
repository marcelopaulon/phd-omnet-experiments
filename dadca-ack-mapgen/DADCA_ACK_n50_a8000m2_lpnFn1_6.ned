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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=28.84,43.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=89.62,165.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=137.27,213.50");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=198.88,201.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=24.35,183.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=217.22,318.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=68.78,278.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=84.51,372.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=181.44,366.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=147.74,292.57");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=220.98,439.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=204.13,504.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=303.16,502.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=282.16,322.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=266.37,255.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=384.74,342.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=442.94,327.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=417.54,241.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=375.38,194.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=446.51,164.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=529.72,137.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=531.07,68.73");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=593.32,47.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=386.15,74.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=448.74,37.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=578.41,203.05");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=634.07,248.37");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=581.66,284.39");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=640.35,145.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=708.36,78.55");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=783.94,22.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=724.57,197.18");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=791.37,231.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=715.13,281.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=872.46,244.98");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=958.46,206.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=929.72,130.05");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=964.44,45.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1082.76,156.02");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1134.59,194.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1153.67,98.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1035.42,108.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1114.46,26.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1217.91,161.95");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1020.92,190.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=874.78,154.36");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=869.95,56.09");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=792.66,98.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1261.66,113.92");
    }

}

