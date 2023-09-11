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
        @display("i=misc/sensor,p=69.97,152.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=157.11,207.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=173.62,136.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=175.90,64.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=79.03,8.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=70.13,219.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=151.43,329.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=212.82,281.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=239.65,217.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=289.19,158.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=290.74,64.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=236.31,35.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=301.73,231.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=335.78,324.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=460.01,314.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=422.94,227.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=367.71,261.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=527.01,252.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=610.85,302.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=526.57,340.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=565.49,195.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=643.11,243.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=625.57,158.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=610.36,85.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=502.65,125.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=738.77,190.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=794.11,165.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=705.79,140.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=705.89,287.66");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=565.45,18.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=648.20,38.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=725.06,5.14");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=781.32,38.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=844.75,2.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=834.07,243.12");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=928.85,141.32");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=853.54,175.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=939.87,6.51");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=894.23,244.56");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=908.84,315.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=992.43,272.69");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=820.34,409.28");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=790.91,467.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=847.86,509.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=825.38,612.86");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=766.17,537.32");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=737.00,592.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=671.65,615.49");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=757.60,681.66");
    }

}

