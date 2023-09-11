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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1.57,111.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=9.29,179.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=30.63,4.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=51.97,63.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=126.72,16.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=69.75,174.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=135.93,152.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=34.41,246.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=149.12,232.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=155.66,305.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=183.63,366.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=213.61,176.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=238.53,241.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=313.53,205.87");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=235.17,105.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=330.04,121.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=333.52,25.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=242.10,5.99");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=183.80,68.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=75.81,312.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=281.64,292.16");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=356.92,266.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=310.43,350.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=377.58,347.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=472.54,290.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=505.48,375.54");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=578.72,430.35");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=663.08,477.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=606.24,515.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=537.44,490.46");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=477.56,532.86");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=552.41,598.01");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=518.07,656.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=504.87,752.03");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=473.55,696.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=387.46,606.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=376.85,699.35");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=440.58,776.79");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=582.86,720.63");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=636.01,820.99");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=694.53,794.52");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=656.35,739.76");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=699.89,683.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=656.87,637.36");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=339.03,759.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=599.81,660.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=620.67,303.30");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=565.72,347.44");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=482.44,443.95");
    }

}

