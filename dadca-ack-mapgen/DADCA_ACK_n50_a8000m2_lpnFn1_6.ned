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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=110.43,6.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=11.43,75.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=219.27,113.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=269.36,71.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=322.15,155.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=251.97,187.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=402.15,219.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=399.48,158.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=474.32,205.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=561.77,226.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=526.70,309.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=561.49,389.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=496.41,398.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=481.66,473.87");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=414.18,459.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=331.96,415.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=245.68,432.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=204.27,521.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=134.63,488.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=46.20,444.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=174.90,397.18");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=86.92,397.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=82.79,332.81");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=151.84,309.07");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=284.73,525.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=239.07,612.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=365.94,607.11");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=315.19,473.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=371.29,525.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=440.84,696.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=462.24,629.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=431.26,577.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=513.95,560.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=582.80,512.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=254.75,683.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=246.05,762.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=162.42,710.71");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=312.89,755.36");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=228.95,847.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=298.26,850.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=401.96,746.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=563.12,648.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=643.83,526.66");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=717.73,503.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=673.95,648.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=761.56,666.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=621.17,592.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=785.83,507.42");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=861.24,479.70");
    }

}

