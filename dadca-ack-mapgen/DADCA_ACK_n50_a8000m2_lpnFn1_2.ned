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
        @display("i=misc/sensor,p=161.45,168.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=162.67,95.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=123.33,10.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=59.41,19.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=188.22,11.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=240.64,133.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=244.31,40.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=356.60,20.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=432.96,57.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=305.52,109.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=156.14,280.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=90.75,244.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=53.84,183.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=82.90,332.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=66.30,406.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=133.12,474.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=112.17,539.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=187.89,593.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=270.71,581.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=300.07,517.00");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=359.53,489.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=344.36,416.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=423.32,417.23");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=433.97,530.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=514.13,482.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=572.45,515.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=497.51,401.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=606.54,418.24");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=642.75,489.91");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=548.37,621.92");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=561.94,703.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=494.12,657.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=365.87,560.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=442.72,599.95");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=382.46,620.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=395.05,744.00");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=461.87,740.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=477.95,843.40");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=548.92,821.42");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=397.73,862.60");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=448.90,941.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=533.31,936.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=592.76,950.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=388.45,956.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=444.49,1041.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=459.49,1110.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=519.30,1086.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=446.33,1185.00");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=489.89,1263.47");
    }

}

