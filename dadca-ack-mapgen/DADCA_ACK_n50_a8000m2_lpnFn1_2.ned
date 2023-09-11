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

network DADCA_ACK_n50_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=62.26,177.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=51.53,250.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=114.53,305.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=36.22,319.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=160.99,387.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=70.05,409.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=241.16,365.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=296.91,317.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=342.48,248.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=299.07,179.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=179.47,291.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=241.04,225.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=242.95,442.28");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=311.11,431.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=298.45,491.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=207.11,517.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=140.01,543.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=209.86,583.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=200.20,661.78");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=272.11,625.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=135.72,625.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=149.90,792.12");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=67.22,751.17");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=11.51,723.14");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=34.32,877.07");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=36.84,803.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=106.01,833.77");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=176.42,719.77");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=79.28,646.33");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=32.50,580.91");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=89.95,509.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=1.50,373.55");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=375.23,519.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=465.43,561.82");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=404.35,576.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=516.64,491.16");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=507.72,427.46");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=448.55,472.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=487.54,355.38");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=399.77,366.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=143.14,459.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=114.13,227.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=321.90,551.55");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=352.35,701.04");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=403.23,750.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=276.85,722.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=237.36,787.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=119.61,693.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=426.19,272.17");
    }

}

