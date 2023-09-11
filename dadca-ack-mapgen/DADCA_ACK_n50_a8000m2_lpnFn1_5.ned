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
        @display("i=misc/sensor,p=196.66,88.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=221.05,179.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=128.64,209.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=134.24,305.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=77.87,358.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=243.38,277.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=60.92,194.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=153.00,142.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=343.69,286.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=452.72,330.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=363.42,354.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=289.34,394.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=238.15,362.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=87.63,451.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=70.95,266.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=7.56,106.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=25.57,38.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=16.74,237.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=188.71,235.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=297.79,188.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=375.29,131.10");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=399.77,215.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=441.28,154.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=350.44,70.95");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=427.98,47.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=246.43,14.24");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=272.41,107.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=196.67,430.81");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=176.14,369.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=214.19,524.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=277.76,490.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=89.29,535.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=35.16,566.43");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=78.84,636.67");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=135.54,597.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=210.04,589.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=303.95,567.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=283.39,624.18");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=386.12,441.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=395.71,533.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=396.77,605.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=471.98,573.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=437.77,482.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=501.96,483.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=572.83,493.66");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=617.23,407.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=615.70,325.76");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=589.27,253.87");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=523.59,182.20");
    }

}

