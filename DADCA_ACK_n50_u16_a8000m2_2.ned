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

network DADCA_ACK_n50_u16_a8000m2_2
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
        @display("i=misc/sensor;p=550.46,7828.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1130.57,7759.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1787.82,7777.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2423.82,7916.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3022.60,7922.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3855.14,7809.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5184.84,7764.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=791.68,7543.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2154.62,7603.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2865.18,7558.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3445.59,7671.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5606.92,7658.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=336.42,7502.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1134.34,7283.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2525.81,7420.76");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3579.70,7304.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4027.65,7381.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4403.59,7469.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4744.82,7275.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1774.21,7219.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2102.04,7084.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2586.19,7042.19");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3047.61,7200.39");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5281.43,7095.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=319.59,6964.74");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=870.46,6940.07");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1606.52,6911.76");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3459.14,6949.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3858.98,6800.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4302.67,6905.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5642.50,6803.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=133.59,6519.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=694.25,6615.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1315.73,6545.83");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1692.95,6563.74");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2465.76,6582.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3151.42,6728.22");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4876.48,6723.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5275.52,6690.64");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2092.78,6408.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3688.58,6319.04");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4638.67,6413.09");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5399.27,6306.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5988.53,6424.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=779.01,6251.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1803.85,6190.31");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2359.68,6172.06");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3083.75,6178.10");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4288.30,6244.93");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5089.01,6114.18");
    }

}

