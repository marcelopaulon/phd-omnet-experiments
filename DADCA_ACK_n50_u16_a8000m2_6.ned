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

network DADCA_ACK_n50_u16_a8000m2_6
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
        @display("i=misc/sensor;p=850.34,7917.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1829.63,7787.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2190.83,7869.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2878.73,7992.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3820.26,7901.71");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5348.85,7956.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=208.35,7663.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2203.94,7511.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3253.31,7601.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5238.55,7613.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5830.49,7565.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1133.16,7481.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1806.34,7332.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3857.41,7369.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4218.99,7427.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4792.69,7319.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=274.82,7150.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=632.42,7151.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2261.82,7134.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2679.57,7086.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3121.94,7256.21");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3578.90,7098.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4295.57,7073.48");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5107.41,7164.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5538.06,7253.23");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=191.90,6787.44");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1673.57,6890.99");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3265.98,6936.43");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3948.90,7003.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4679.66,6899.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=591.50,6671.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1056.92,6692.38");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2044.46,6738.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2807.83,6736.15");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3473.81,6528.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3920.87,6638.54");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4359.53,6536.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5287.86,6729.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=60.68,6394.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=457.50,6269.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1567.78,6380.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2665.05,6354.90");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3079.79,6347.88");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5799.82,6293.97");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2.20,6043.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=949.33,6172.30");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3757.96,6213.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4774.18,6168.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5360.34,6204.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=685.02,5878.77");
    }

}

