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

network DADCA_ACK_n50_a8000m2_3
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
        @display("i=misc/sensor;p=193.67,7980.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=299.61,7999.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=429.59,7991.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=668.89,7961.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=899.61,7961.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=954.37,7991.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=242.28,7930.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=482.73,7951.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=580.99,7920.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=718.32,7922.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=775.78,7944.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=128.13,7895.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=208.36,7870.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=358.86,7907.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=518.03,7870.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=632.98,7873.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=781.28,7876.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=856.74,7883.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=959.15,7904.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=31.04,7833.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=298.40,7862.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=461.65,7838.49");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=968.96,7837.49");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=143.63,7814.03");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=213.99,7798.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=381.35,7821.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=551.07,7800.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=793.35,7814.29");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=886.17,7786.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=20.44,7746.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=96.80,7762.29");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=298.60,7768.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=476.24,7769.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=659.24,7778.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=736.23,7773.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=822.48,7753.94");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=159.19,7739.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=255.59,7707.78");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=399.29,7731.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=554.85,7739.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=635.01,7704.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=951.80,7737.42");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=51.41,7685.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=183.58,7673.52");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=338.28,7667.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=479.15,7685.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=736.08,7686.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=841.95,7657.27");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=6.43,7617.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=80.98,7613.41");
    }

}

