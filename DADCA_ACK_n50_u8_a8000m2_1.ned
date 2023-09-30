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

network DADCA_ACK_n50_u8_a8000m2_1
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
        @display("i=misc/sensor;p=1453.67,7827.76");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3319.97,7989.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5006.73,7931.45");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5412.77,7787.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5839.66,7765.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=30.68,7582.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=851.05,7624.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1817.48,7529.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2261.08,7667.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2802.41,7578.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3603.30,7656.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5126.96,7529.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1167.80,7384.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2523.27,7312.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3312.06,7352.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4645.53,7358.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=540.14,7167.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1858.07,7173.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2856.07,7042.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3961.28,7250.73");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5086.89,7110.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5744.88,7145.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1368.22,6917.59");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2196.80,6828.45");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3246.96,6869.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3678.44,6957.55");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4268.66,6957.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=997.20,6742.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1966.68,6555.78");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2595.43,6728.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3981.53,6557.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4450.03,6646.66");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5044.37,6522.23");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5724.72,6644.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=729.70,6446.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1194.98,6326.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2818.15,6354.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3255.21,6413.15");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4411.70,6280.87");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=718.17,6066.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1560.61,6265.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4877.70,6071.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5252.98,6235.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5650.47,6180.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1774.00,5947.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2139.76,6003.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3760.60,5973.64");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5536.38,5829.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5914.40,5918.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=874.93,5526.76");
    }

}

