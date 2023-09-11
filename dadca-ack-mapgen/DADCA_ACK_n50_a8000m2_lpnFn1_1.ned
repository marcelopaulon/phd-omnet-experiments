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
        @display("i=misc/sensor,p=53.71,11.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=24.50,95.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=167.97,39.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=224.11,75.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=89.09,170.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=293.58,53.82");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=344.00,17.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=429.07,41.62");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=507.79,52.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=540.93,118.25");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=456.40,125.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=399.34,166.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=306.42,202.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=296.85,123.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=489.99,225.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=560.76,206.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=426.03,250.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=419.05,325.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=512.98,310.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=376.92,82.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=670.17,122.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=664.94,189.64");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=608.67,38.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=689.31,26.42");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=764.94,58.30");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=321.76,306.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=261.88,328.04");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=305.94,406.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=246.32,466.35");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=197.99,386.81");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=177.05,469.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=244.79,557.38");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=186.74,585.69");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=103.10,496.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=61.66,448.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=24.01,389.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=9.17,322.23");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=68.92,345.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=349.21,479.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=360.81,552.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=407.16,590.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=372.38,397.48");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=436.23,432.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=460.02,376.65");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=526.83,425.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=647.44,346.36");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=662.90,277.25");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=714.62,409.58");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=682.65,460.87");
    }

}

