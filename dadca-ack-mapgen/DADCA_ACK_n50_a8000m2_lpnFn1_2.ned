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
        @display("i=misc/sensor,p=132.80,169.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=215.66,176.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=289.13,216.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=204.04,85.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=46.89,237.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=4.54,117.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=30.29,47.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=59.31,158.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=288.84,155.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=188.71,246.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=144.24,290.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=353.14,251.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=424.13,253.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=387.87,195.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=260.15,281.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=126.88,393.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=107.58,226.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=316.92,394.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=357.04,336.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=326.82,492.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=341.19,560.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=252.64,517.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=235.50,417.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=287.14,337.51");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=421.59,364.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=516.84,350.72");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=496.50,272.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=562.38,392.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=607.23,438.12");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=687.84,474.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=684.38,354.82");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=747.99,334.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=738.88,396.68");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=836.56,363.54");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=847.24,443.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=894.22,499.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=930.89,429.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1001.19,479.93");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1024.43,573.50");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1115.04,550.19");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1077.89,488.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=961.77,589.48");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1142.75,449.91");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1131.30,353.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1071.17,361.37");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=978.06,388.93");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=986.28,311.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1039.79,272.99");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=978.42,196.27");
    }

}

