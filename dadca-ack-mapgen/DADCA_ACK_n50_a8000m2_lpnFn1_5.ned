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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=138.99,13.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=228.14,22.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=203.50,87.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=257.36,127.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=245.25,186.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=221.41,248.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=284.14,277.33");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=225.12,320.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=233.28,405.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=288.79,357.28");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=387.24,357.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=319.83,440.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=379.23,450.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=216.70,504.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=237.40,561.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=344.01,513.00");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=379.22,569.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=450.35,567.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=465.96,478.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=509.03,547.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=467.03,385.44");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=151.20,289.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=163.24,358.40");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=151.29,455.97");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=90.02,445.49");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=30.08,475.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=6.84,389.30");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=73.20,385.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=131.84,551.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=492.70,305.30");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=567.54,359.61");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=578.62,433.58");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=623.29,473.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=699.30,434.54");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=782.66,435.38");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=625.07,321.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=585.04,253.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=525.12,219.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=703.65,287.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=654.40,234.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=627.91,170.80");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=699.15,123.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=771.75,132.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=722.20,207.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=841.66,278.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=863.80,343.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=905.39,405.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=969.22,435.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=979.39,322.05");
    }

}

