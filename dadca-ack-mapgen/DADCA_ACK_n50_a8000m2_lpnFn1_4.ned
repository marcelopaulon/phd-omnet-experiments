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
        @display("i=misc/sensor,p=132.23,158.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=122.60,3.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=59.98,5.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=11.98,122.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=56.70,179.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=236.28,146.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=298.56,189.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=364.42,208.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=426.75,242.88");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=462.90,193.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=495.25,141.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=541.34,204.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=620.98,139.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=693.96,141.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=732.69,198.93");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=654.51,63.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=576.47,55.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=486.32,70.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=427.26,97.06");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=374.25,54.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=299.94,91.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=253.37,21.73");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=298.36,280.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=246.64,230.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=209.71,278.49");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=270.20,348.54");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=215.06,396.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=291.58,425.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=158.43,423.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=80.81,382.30");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=9.94,377.63");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=359.00,292.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=405.63,354.27");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=473.90,366.72");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=415.36,417.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=516.10,314.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=544.04,422.59");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=490.24,475.92");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=365.88,524.29");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=445.65,590.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=442.57,526.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=313.69,612.39");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=290.80,671.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=348.81,698.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=419.71,702.39");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=308.74,800.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=247.28,730.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=305.82,860.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=406.22,775.61");
    }

}

