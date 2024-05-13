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

network DADCA_ACK_n50_u2_a8000m2_1
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
        @display("i=misc/sensor;p=480.20,7989.76");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=990.87,7822.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1959.26,7946.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3406.27,7930.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4549.58,7979.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5772.47,7754.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=90.88,7636.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=699.74,7620.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1646.90,7610.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2527.87,7656.58");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2875.04,7703.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4218.72,7736.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5069.67,7552.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=501.39,7302.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1065.27,7373.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2057.66,7289.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3124.27,7411.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3629.19,7470.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4414.96,7354.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2455.97,7245.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5666.27,7191.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=92.54,6995.38");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=984.73,6993.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2763.22,6901.75");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3419.24,6851.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3904.57,6886.49");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5304.82,6931.18");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5952.09,6854.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=447.20,6760.44");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=870.25,6639.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1939.46,6627.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4391.26,6552.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4899.36,6645.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5271.79,6536.55");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1254.47,6445.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3477.32,6466.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3939.53,6471.22");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=192.00,6229.43");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1823.26,6111.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2269.67,6077.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2911.26,6264.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3319.86,6120.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4212.55,6126.88");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4705.69,6072.78");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5521.57,6219.74");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=885.73,5995.93");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1226.98,5870.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2666.03,5940.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5429.96,5837.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5943.31,5998.28");
    }

}

