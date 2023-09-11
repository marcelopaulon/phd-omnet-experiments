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
        @display("i=misc/sensor,p=60.14,167.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=136.90,185.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=198.93,212.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=182.12,286.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=127.65,365.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=59.83,397.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=106.74,439.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=149.41,490.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=92.41,555.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=239.40,435.25");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=316.77,405.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=295.64,496.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=363.59,482.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=435.91,514.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=375.13,391.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=373.27,318.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=458.79,410.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=537.48,396.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=507.00,464.82");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=603.87,472.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=660.41,439.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=648.55,366.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=679.03,538.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=627.34,607.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=644.81,680.58");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=556.99,718.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=489.87,725.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=528.55,774.30");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=613.62,805.35");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=574.52,884.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=635.18,898.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=674.28,809.21");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=686.36,743.83");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=745.25,860.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=418.84,777.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=386.55,686.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=318.65,830.43");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=353.39,767.62");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=372.29,865.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=399.42,944.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=276.60,885.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=272.85,962.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=314.04,1011.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=190.57,941.64");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=151.62,862.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=109.74,817.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=50.22,834.40");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=105.79,908.18");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=142.11,1045.84");
    }

}

