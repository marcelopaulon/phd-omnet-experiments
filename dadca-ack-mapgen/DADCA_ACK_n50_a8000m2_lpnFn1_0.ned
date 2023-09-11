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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=147.08,17.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=5.00,49.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=23.68,186.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=81.61,224.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=103.55,308.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=125.50,159.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=211.97,254.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=63.25,25.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=246.03,98.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=331.49,109.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=314.15,49.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=393.87,118.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=399.59,37.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=456.18,7.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=524.94,94.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=485.41,148.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=491.14,218.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=419.57,234.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=455.76,325.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=340.33,274.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=256.21,298.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=297.09,400.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=218.64,419.20");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=202.70,358.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=230.14,493.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=141.68,392.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=338.63,335.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=158.93,504.78");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=79.68,558.84");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=16.94,628.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=110.90,665.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=166.34,715.00");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=247.44,639.24");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=310.78,683.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=155.49,609.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=319.97,573.77");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=392.97,578.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=437.23,626.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=479.36,710.29");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=373.97,468.42");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=429.89,407.05");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=580.82,338.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=533.82,397.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=549.91,466.65");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=598.48,506.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=673.54,544.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=698.77,465.69");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=788.27,541.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=847.13,564.30");
    }

}

