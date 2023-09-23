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

network DADCA_ACK_n50_u4_a8000m2_3
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
        @display("i=misc/sensor;p=541.29,7785.41");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1082.76,7817.89");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1654.99,7857.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2014.94,7830.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3453.82,7974.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4425.01,7908.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2610.49,7662.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3066.70,7635.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3676.44,7590.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4842.57,7727.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5248.73,7733.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5581.91,7586.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=219.07,7415.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1236.16,7468.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1927.21,7366.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3280.53,7320.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4234.53,7339.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4640.58,7383.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=714.88,7174.73");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1551.89,7247.75");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2265.51,7047.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2849.13,7169.11");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5177.35,7211.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5764.68,7093.52");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1158.41,6956.98");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1739.54,6895.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3408.43,6944.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4125.63,6934.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4788.63,6994.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5147.77,6819.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=257.67,6740.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=746.36,6633.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1426.12,6547.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2625.29,6607.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4443.35,6746.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5680.64,6520.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3226.10,6480.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3956.72,6404.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4319.84,6343.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4753.45,6502.74");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=756.22,6185.87");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1260.07,6137.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1917.56,6224.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2697.83,6083.94");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3655.78,6178.41");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5008.15,6247.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5411.23,6172.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5755.17,6025.55");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=261.00,5925.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=929.08,5792.24");
    }

}

