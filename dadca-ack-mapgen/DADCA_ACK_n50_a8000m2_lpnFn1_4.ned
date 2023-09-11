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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=80.85,27.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=54.89,147.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=8.56,63.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=30.54,254.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=63.50,310.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=42.02,402.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=116.99,351.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=175.59,417.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=189.83,338.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=205.01,487.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=272.44,431.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=328.92,457.61");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=355.92,403.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=367.05,580.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=297.17,564.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=472.57,549.79");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=476.92,626.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=489.45,715.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=570.83,756.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=557.21,814.92");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=543.69,649.62");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=623.49,614.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=717.48,597.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=776.58,535.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=838.15,588.40");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=802.69,669.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=841.35,742.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=832.46,838.05");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=812.58,931.69");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=890.69,882.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=866.11,659.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=786.07,788.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=708.61,668.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=650.87,750.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=622.17,679.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=674.62,848.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=610.69,850.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=711.96,748.78");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=928.41,768.57");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=899.51,715.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=992.85,666.03");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1048.19,709.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=992.32,551.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1024.08,471.10");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1054.69,559.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1082.10,656.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1073.69,805.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1011.71,784.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1164.01,768.26");
    }

}

