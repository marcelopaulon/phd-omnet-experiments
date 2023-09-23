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

network DADCA_ACK_n50_u4_a8000m2_0
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
        @display("i=misc/sensor;p=1616.58,7752.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2868.23,7970.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4393.52,7973.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5225.97,7961.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2235.55,7683.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2874.57,7569.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3520.54,7681.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4038.10,7582.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4698.50,7739.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5730.97,7673.45");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1245.48,7457.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1920.30,7468.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2444.51,7385.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4917.48,7397.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5262.77,7476.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=188.85,7072.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=587.67,7220.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1423.84,7049.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1875.07,7097.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2284.85,7060.93");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3264.69,7188.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4250.42,7255.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5124.54,7064.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5648.03,7037.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=576.42,6804.61");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=934.67,6767.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3319.64,6825.83");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3744.41,6975.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4760.88,6854.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=166.51,6719.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1573.62,6708.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1991.07,6713.21");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2923.02,6648.05");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4210.78,6719.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5255.06,6719.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=414.79,6435.96");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=850.50,6377.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1215.20,6431.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2421.12,6472.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3872.96,6396.49");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4537.72,6336.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4924.46,6332.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5425.45,6350.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=760.50,6035.60");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1545.65,6267.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2089.22,6059.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2521.88,6115.00");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2903.04,6237.26");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3416.64,6185.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4177.78,6021.69");
    }

}

