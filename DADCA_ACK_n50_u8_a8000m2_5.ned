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

network DADCA_ACK_n50_u8_a8000m2_5
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
        @display("i=misc/sensor;p=3256.69,7986.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3781.49,7932.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4321.52,7929.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5490.68,7929.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5980.78,7814.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2808.32,7680.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3381.91,7560.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5165.55,7722.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5577.58,7560.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2929.12,7302.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3892.36,7445.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4277.66,7423.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4739.26,7458.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4088.11,7041.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4553.91,7067.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4975.61,7142.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5414.87,7090.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5966.09,7202.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2557.70,6956.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3613.95,6920.75");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2793.33,6649.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3158.40,6757.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3964.91,6608.05");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4395.64,6554.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4757.39,6525.74");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5524.86,6757.64");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2504.81,6276.05");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3615.06,6358.46");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2020.75,6256.67");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3198.44,6215.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4251.71,6218.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4736.09,6126.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5137.79,6089.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5490.51,6152.81");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1507.75,6008.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1972.92,5780.20");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3394.40,5920.23");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=665.17,5647.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1236.57,5607.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2391.76,5767.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2856.12,5717.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3719.81,5715.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4358.63,5742.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4713.57,5715.06");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1752.72,5466.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2408.18,5297.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3109.98,5320.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4254.14,5367.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5273.82,5441.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5753.12,5507.56");
    }

}

