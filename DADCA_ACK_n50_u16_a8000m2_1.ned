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

network DADCA_ACK_n50_u16_a8000m2_1
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
        @display("i=misc/sensor;p=454.72,7767.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1895.35,7853.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3933.20,7942.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5823.86,7863.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=108.83,7685.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=790.89,7575.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1491.71,7544.77");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2664.62,7539.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3120.01,7710.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3586.59,7701.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4429.34,7646.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4989.03,7721.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1897.86,7406.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4097.84,7491.19");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4934.40,7292.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5419.20,7436.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=401.69,7209.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1056.66,7069.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1498.28,7146.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2414.35,7171.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3484.48,7076.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3919.10,7068.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4552.30,7218.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5636.19,7010.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5957.00,7247.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2080.04,6979.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2984.07,6804.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4558.61,6789.36");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5040.69,6858.67");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=581.73,6572.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1498.16,6697.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2604.81,6734.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3515.63,6565.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4206.33,6711.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5681.46,6579.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1062.55,6438.23");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2078.84,6373.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3077.36,6455.68");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4045.37,6364.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=629.40,6097.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1310.03,6076.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2416.21,6143.54");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3598.73,6138.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4756.54,6192.54");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5991.90,6240.02");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=95.01,5890.98");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=922.87,5788.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1682.27,5884.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3062.29,6000.00");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3919.60,5927.15");
    }

}

