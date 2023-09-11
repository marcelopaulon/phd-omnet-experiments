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

network DADCA_ACK_n50_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=128.63,173.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=62.63,199.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=15.97,154.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=75.87,32.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=21.74,72.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=81.13,286.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=10.96,343.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=51.03,409.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=115.48,336.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=158.20,57.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=248.72,79.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=221.43,139.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=286.13,149.00");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=298.88,230.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=346.94,150.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=439.08,114.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=505.66,113.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=521.45,194.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=580.18,245.19");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=604.11,174.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=462.67,256.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=552.59,299.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=624.79,361.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=615.42,461.04");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=546.54,392.71");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=545.54,480.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=463.92,432.41");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=477.47,344.89");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=409.58,376.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=339.08,321.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=421.32,483.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=712.96,389.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=694.14,303.60");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=688.18,225.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=567.97,70.89");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=388.26,271.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=755.04,290.72");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=750.37,227.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=855.82,322.95");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=905.62,370.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=869.56,445.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=780.50,432.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=458.36,18.33");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=396.21,64.97");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=671.10,433.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=729.05,467.42");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=673.86,544.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=572.09,542.50");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=536.15,598.03");
    }

}

