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

network DADCA_ACK_n50_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=40.90,60.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=181.65,21.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=49.55,209.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=130.67,152.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=96.41,19.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=187.92,178.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=256.53,212.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=246.99,307.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=168.65,338.25");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=74.89,367.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=143.87,437.93");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=147.61,508.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=62.65,475.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=206.24,401.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=315.79,458.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=335.58,554.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=414.65,573.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=326.77,639.17");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=243.31,556.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=389.53,420.88");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=432.50,369.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=499.45,353.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=537.34,277.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=580.30,322.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=561.83,400.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=585.67,457.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=496.49,441.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=488.70,541.16");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=567.49,571.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=478.48,615.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=425.18,498.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=518.34,677.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=486.92,740.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=407.91,800.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=391.63,702.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=238.66,656.86");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=366.17,357.99");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=434.85,307.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=556.21,749.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=512.10,828.14");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=546.77,902.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=602.51,936.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=687.61,967.64");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=774.24,1013.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=739.65,1095.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=672.66,1088.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=804.88,956.83");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=841.85,1039.44");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=740.98,882.99");
    }

}

