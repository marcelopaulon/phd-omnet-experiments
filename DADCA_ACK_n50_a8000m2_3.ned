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
        @display("i=misc/sensor;p=129.89,7951.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=742.59,7803.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1187.93,7908.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1624.41,7953.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2233.86,7893.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2784.47,7855.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4151.11,7948.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4573.69,7777.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5177.38,7846.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5965.45,7932.50");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=83.53,7559.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3648.22,7544.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5546.23,7722.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1946.87,7374.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2575.20,7413.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2988.61,7432.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4202.52,7344.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4650.02,7401.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5098.63,7446.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=300.56,7222.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1289.97,7163.02");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3857.62,7106.83");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5462.01,7133.71");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=517.22,6893.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2119.59,6898.20");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2483.75,6828.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2891.82,6830.76");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3394.07,6801.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4383.59,6896.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5107.72,6846.79");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5825.70,6886.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=751.22,6576.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1290.02,6662.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1713.77,6639.84");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=329.82,6421.34");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2153.40,6405.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2560.68,6361.90");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3166.04,6436.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4002.52,6423.33");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4652.81,6279.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5299.98,6367.49");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5732.81,6252.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=606.76,6169.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=942.23,6019.93");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1466.99,6060.25");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1945.69,6086.94");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2635.51,5960.18");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3104.81,5958.25");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3716.99,6127.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5397.04,5931.92");
    }

}

